package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.commands;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus;
//import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.pixy.PixyBlock;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kD;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kI;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kP;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.LEFT;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.RIGHT;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.ROBOT_CIRCUMFERENCE;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.TURN_SPEED;

/**
 * Code for using encoders
 */
public class Encoders extends LinearOpMode {

    private HardwareZeus zeus = new HardwareZeus();
    private ElapsedTime timer = new ElapsedTime();

//    private PixyBlock block1;
    private double goal = 0.0;
    private double lastError, integralError = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        zeus.init(hardwareMap);
    }

    public void encoderDrive(double speed,
                             double distance) {
        int newTarget;

//        block1 = zeus.pixy.getBiggestBlock(1);

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

//                double error = goal - block1.x;
//                integralError += error;
//                double deltaError = error - lastError;
//
//                double Pterm = kP * error;
//                double Iterm = kI * integralError;
//                double Dterm = kD * deltaError;
//
//                double correction = Pterm + Iterm + Dterm;
//                correction = Math.min(0.4, correction);
//                correction = Math.max(-0.4, correction);
//
//                zeus.motorRight.setPower(speed + correction);
//                zeus.motorLeft.setPower(speed - correction);
//
//                lastError = error;
//
//                telemetry.addData("Motor1", zeus.motorRight.getCurrentPosition());
//                telemetry.addData("Motor2", zeus.motorLeft.getCurrentPosition());
//                telemetry.update();
//
                telemetry.addData("Hi", "Alison");
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
}