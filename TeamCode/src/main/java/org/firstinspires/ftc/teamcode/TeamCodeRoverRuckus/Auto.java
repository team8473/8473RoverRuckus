package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.LEFT;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.RIGHT;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.ROBOT_CIRCUMFERENCE;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kD;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kI;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kP;

@Autonomous (name = "Auto", group = "Test")
public class Auto extends LinearOpMode {

    private HardwareZeus zeus = new HardwareZeus();
    private ElapsedTime timer = new ElapsedTime();

    private double lastError, integralError;
    private double globalAngle;

    private Orientation angles, lastAngles = null;


    @Override
    public void runOpMode() {
        zeus.init(hardwareMap);

        zeus.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zeus.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(10);
        zeus.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        zeus.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

//        encoderTurn(180, RIGHT);
//        gyroTurn(TURN_SPEED, 180, LEFT);
        gyroTurnTest(TURN_SPEED, 155, LEFT);

    }

    public void encoderTurn(double turnAngle,
                             String direction) {

        double circleFraction = turnAngle / 360;
        double inches = (circleFraction * ROBOT_CIRCUMFERENCE);

        switch (direction) {
            case RIGHT :
                encoderDrive(TURN_SPEED, inches);
                break;
            case LEFT :
                encoderDrive(-TURN_SPEED, inches);
                break;
        }
    }

    public void gyroTurn(double speed,
                          double angle,
                           @NonNull String direction) {
        resetAngle();

        switch(direction){
            case RIGHT:
                resetAngle();
                while (opModeIsActive() && angle > getAngle()) {
                    resetAngle();

                    telemetry.addData("Angle 1", getAngle());
                    telemetry.update();

                    double error = angle - angles.firstAngle;
                    integralError += error;
                    double deltaError = error - lastError;

                    double Pterm = kP * error;
                    double Iterm = kI * integralError;
                    double Dterm = kD * deltaError;

                    double correction = Pterm + Iterm + Dterm;
                    correction = Math.min(0.2, correction);
                    correction = Math.max(-0.2, correction);

                    zeus.motorRight.setPower(-speed - correction);
                    zeus.motorLeft.setPower(speed - correction);

                    lastError = error;

                }
                break;
            case LEFT:
                resetAngle();
                while (opModeIsActive() && -angle < getAngle()) {
                    resetAngle();

                    double error = angle - angles.firstAngle;
                    integralError += error;
                    double deltaError = error - lastError;

                    double Pterm = kP * error;
                    double Iterm = kI * integralError;
                    double Dterm = kD * deltaError;

                    double correction = Pterm + Iterm + Dterm;
                    correction = Math.min(0.4, correction);
                    correction = Math.max(-0.4, correction);

                    zeus.motorRight.setPower(-speed - correction);
                    zeus.motorLeft.setPower(speed - correction);

                    lastError = error;

                }
                break;
            default:
                break;
        }
    }

    private void gyroTurnTest(double speed,
                               double angle,
                                @NonNull String direction) {
        resetAngle();

        switch(direction){
            case RIGHT:
                resetAngle();
                while (angle > angles.firstAngle) {
                    resetAngle();
                    zeus.motorRight.setPower(-speed);
                    zeus.motorLeft.setPower(speed);
                }
                zeus.motorRight.setPower(0.0);
                zeus.motorLeft.setPower(0.0);
                break;
            case LEFT:
                resetAngle();
                while (-angle < angles.firstAngle) {
                    resetAngle();
                    zeus.motorRight.setPower(speed);
                    zeus.motorLeft.setPower(-speed);
                }
                zeus.motorRight.setPower(0.0);
                zeus.motorLeft.setPower(0.0);
                telemetry.addData("First Angle", angles.firstAngle);
                telemetry.update();
                sleep(10000);
                break;
            default:
                break;
        }
    }

    private void encoderDrive(double speed,
                               double distance) {
        int newTarget;

        if (opModeIsActive()) {
            newTarget = zeus.motorLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

            if (speed == TURN_SPEED) {
                zeus.motorRight.setTargetPosition(newTarget);
                zeus.motorLeft.setTargetPosition(-newTarget);
            } else if (speed == -TURN_SPEED) {
                zeus.motorRight.setTargetPosition(-newTarget);
                zeus.motorLeft.setTargetPosition(newTarget);
            } else {
                zeus.motorRight.setTargetPosition(newTarget);
                zeus.motorLeft.setTargetPosition(newTarget);
            }

            zeus.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            zeus.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            timer.reset();
            zeus.motorRight.setPower(Math.abs(speed));
            zeus.motorLeft.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (timer.seconds() < 10) &&
                    (zeus.motorRight.isBusy() && zeus.motorLeft.isBusy())) {

                telemetry.addData("Motor1", zeus.motorRight.getCurrentPosition());
                telemetry.addData("Motor2", zeus.motorLeft.getCurrentPosition());
                telemetry.update();

            }
            zeus.motorRight.setPower(0);
            zeus.motorLeft.setPower(0);

            zeus.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            zeus.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void resetAngle() {
        lastAngles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {

        angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        }
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}
