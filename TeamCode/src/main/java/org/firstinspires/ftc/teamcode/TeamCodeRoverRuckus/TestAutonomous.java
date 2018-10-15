package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.pixy.PixyBlock;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kD;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kI;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kP;

@Autonomous (name = "Test", group = "Main")
@Disabled
public class TestAutonomous extends LinearOpMode {

    private HardwareZeus zeus = new HardwareZeus();
    private ElapsedTime timer = new ElapsedTime();


    private double lastError, integralError;
    private double goalAngle, xGoal;
    private PixyBlock block1;

    private Orientation angles = null;

    @Override
    public void runOpMode() throws InterruptedException {

        zeus.init(hardwareMap);

        angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        goalAngle = angles.firstAngle;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        zeus.imu = hardwareMap.get(BNO055IMU.class, "imu");
        zeus.imu.initialize(parameters);

        zeus.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zeus.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(10);
        zeus.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        zeus.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        pixyDrive(DRIVE_SPEED, 20);

        if (angles.firstAngle < -5) {
//            gyroDrive(DRIVE_SPEED, 10, Whatever angle it needs to be);
            zeus.servo1.setPosition(.5);
            zeus.servo2.setPosition(.5);
        } else if (angles.firstAngle > 5) {
//            gyroDrive(DRIVE_SPEED, 10, Whatever angle it needs to be);
            zeus.servo1.setPosition(.5);
            zeus.servo2.setPosition(.5);
        } else {
            gyroDrive(DRIVE_SPEED, 10, goalAngle);
            zeus.servo1.setPosition(.5);
            zeus.servo2.setPosition(.5);
        }
    }

    private void gyroDrive(double speed,
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

    private void pixyDrive(double speed,
                             double distance) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newRightTarget = zeus.motorRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = zeus.motorLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

            zeus.motorRight.setTargetPosition(-newRightTarget);
            zeus.motorLeft.setTargetPosition(-newLeftTarget);

            zeus.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            zeus.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            timer.reset();
            zeus.motorRight.setPower(Math.abs(speed));
            zeus.motorLeft.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (zeus.motorRight.isBusy() && zeus.motorLeft.isBusy())) {

                block1 = zeus.pixy.getBiggestBlock(1);

                xGoal = 0.0;

                double error = xGoal - block1.x;
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

                telemetry.addData("Motor 1", zeus.motorRight.getCurrentPosition());
                telemetry.addData("Motor 2", zeus.motorLeft.getCurrentPosition());
                telemetry.addData("Block 1", block1.x);
                telemetry.update();
            }
            zeus.motorRight.setPower(0);
            zeus.motorLeft.setPower(0);

            zeus.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            zeus.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}