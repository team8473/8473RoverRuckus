package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus;
import org.firstinspires.ftc.teamcode.pixy.PixyCam_Analog;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.LEFT;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.RIGHT;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.ROBOT_CIRCUMFERENCE;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kD;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kI;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kP;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.TURN_SPEED;

/**
 * Code for using encoders
 */
@Autonomous(name = "AutoDrive", group = "commands")
@Disabled
public class AutoDrive extends LinearOpMode {

    private PixyCam_Analog pixy = null;
    private HardwareZeus zeus = new HardwareZeus();
    private ElapsedTime timer = new ElapsedTime();

    private double lastError, integralError;
    private double xGoal, block1;

    private Orientation angles = null;

    @Override
    public void runOpMode() throws InterruptedException {
        zeus.init(hardwareMap);
        pixy = new PixyCam_Analog(hardwareMap);
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

    public void pixyDrive(double speed,
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

                block1 = pixy.getAnalogRead();

                xGoal = 1.15;

                double error = block1 - xGoal;
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
                telemetry.addData("Block 1", block1);
                telemetry.update();
            }
            zeus.motorRight.setPower(0);
            zeus.motorLeft.setPower(0);

            zeus.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            zeus.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDrive(double speed,
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
                           String direction) {
        angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double angle1 = angle + 1.0;
        double angle2 = angle - 1.0;

        switch(direction){
            case RIGHT:
                while (angle2 < angles.firstAngle && angles.firstAngle < angle1) {

                    angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    double error = angle - angles.firstAngle;
                    integralError += error;
                    double deltaError = error - lastError;

                    double Pterm = kP * error;
                    double Iterm = kI * integralError;
                    double Dterm = kD * deltaError;

                    double correction = Pterm + Iterm + Dterm;
                    correction = Math.min(0.4, correction);
                    correction = Math.max(-0.4, correction);

                    zeus.motorRight.setPower(speed + correction);
                    zeus.motorLeft.setPower(-speed + correction);

                    lastError = error;

                }
                break;
            case LEFT:
                while (-angle1 < angles.firstAngle && angles.firstAngle < -angle2) {

                    angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

    public void gyroTurnTest(double speed,
                              double angle,
                               String direction) {
        angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double angle1 = angle + 0.5;
        double angle2 = angle - 0.5;

        switch(direction){
            case RIGHT:
                while (angle2 < angles.firstAngle && angles.firstAngle < angle1) {
                    zeus.motorRight.setPower(speed);
                    zeus.motorLeft.setPower(-speed);
                }
                zeus.motorRight.setPower(0.0);
                zeus.motorLeft.setPower(0.0);
                break;
            case LEFT:
                while (-angle1 < angles.firstAngle && angles.firstAngle < -angle2) {
                    zeus.motorRight.setPower(-speed);
                    zeus.motorLeft.setPower(speed);
                }
                zeus.motorRight.setPower(0.0);
                zeus.motorLeft.setPower(0.0);
                break;
            default:
                break;
        }
    }
}