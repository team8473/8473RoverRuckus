package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kD;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kI;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kP;

@Autonomous(name = "Straight", group = "Red")
@Disabled
public class RedStraight extends LinearOpMode {

    private HardwareZeus zeus = new HardwareZeus();
    private ElapsedTime timer = new ElapsedTime();

    private Orientation angles = null;

    private double lastError, integralError;
    private double goalAngle;

    @Override
    public void runOpMode() throws InterruptedException {
        zeus.init(hardwareMap);

        zeus.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zeus.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(10);
        zeus.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        zeus.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        zeus.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        goalAngle = angles.firstAngle;
        gyroDrive(DRIVE_SPEED, 55, goalAngle);
        zeus.servo1.setPosition(CLAW_CLOSED);
        zeus.servo2.setPosition(CLAW_CLOSED);
        telemetry.addData("Servo Position : ", zeus.servo1.getPosition());
        telemetry.update();
        sleep(10000);
    }

    public void gyroDrive(double speed,
                          double distance, double goalAngle) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newRightTarget = zeus.motorRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = zeus.motorLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

            if (speed == TURN_SPEED) {
                zeus.motorRight.setTargetPosition(newRightTarget);
                zeus.motorLeft.setTargetPosition(-newLeftTarget);
            } else if (speed == -TURN_SPEED) {
                zeus.motorRight.setTargetPosition(-newRightTarget);
                zeus.motorLeft.setTargetPosition(newLeftTarget);
            } else {
                zeus.motorRight.setTargetPosition(-newRightTarget);
                zeus.motorLeft.setTargetPosition(-newLeftTarget);
            }

            zeus.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            zeus.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            timer.reset();
            zeus.motorRight.setPower(Math.abs(speed));
            zeus.motorLeft.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (zeus.motorRight.isBusy() && zeus.motorLeft.isBusy())) {

                angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                double error = goalAngle - angles.firstAngle;
                integralError += error;
                double deltaError = error - lastError;

                double Pterm = kP * error;
                double Iterm = kI * integralError;
                double Dterm = kD * deltaError;

                double correction = Pterm + Iterm + Dterm;
                correction = Math.min(0.4, correction);
                correction = Math.max(-0.4, correction);

                zeus.motorRight.setPower(speed + correction);
                zeus.motorLeft.setPower(speed - correction);

                lastError = error;

                telemetry.addData("Motor1", zeus.motorRight.getCurrentPosition());
                telemetry.addData("Motor2", zeus.motorLeft.getCurrentPosition());
                telemetry.addData("heading", angles.firstAngle);
                telemetry.update();
            }
            zeus.motorRight.setPower(0);
            zeus.motorLeft.setPower(0);

            zeus.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            zeus.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
