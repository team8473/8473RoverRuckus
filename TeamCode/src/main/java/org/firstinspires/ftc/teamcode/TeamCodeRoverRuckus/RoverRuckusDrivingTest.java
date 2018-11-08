package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.commands.Wait;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.CLAW_OPEN;

@TeleOp(name = "DrivingTest", group = "Match")
//@Disabled
public class RoverRuckusDrivingTest extends OpMode {

    private HardwareZeus zeus = new HardwareZeus();
    private Wait wait = new Wait();

    private double DRIVE_SPEED;

    private int currentPosition = 1;
    private int currentControl = 0;
    private int currentSpeed = 1;
    private Controls control;
    private Speed speed;

    private static Position[] position = new Position[] {Position.CLAW_CLOSED, Position.CLAW_OPEN};
    private static Controls[] controls = new Controls[] {Controls.NORMAL, Controls.INVERTED};
    private static Speed[] speeds = new Speed[] {Speed.DRIVE_FAST, Speed.DRIVE_SLOW};

    @Override
    public void init() {
        zeus.init(hardwareMap);

        control = controls[currentControl];
        speed = speeds[currentSpeed];
    }

    @Override
    public void start() {
        zeus.runtime.reset();
    }

    @Override
    public void loop() {

        //Claw
        if (gamepad2.a) {
           clawCycle();
        }

        //Lift
        float lift = gamepad2.left_stick_y;
        lift = Range.clip(-lift, -1, 1);
        zeus.motorLift.setPower(Range.clip(lift, -.6, .6));

        //Driving
        float right = -gamepad1.right_stick_y;
        float left = -gamepad1.left_stick_y;

        right = Range.clip(-right, -1, 1);
        left = Range.clip(-left, -1, 1);

        if (gamepad1.right_bumper) {
            speedCycle();
        }

        if (gamepad1.left_bumper) {
            controlCycle();
        }

        switch (control) {
            case NORMAL:
                zeus.motorRight.setPower(Range.clip(right, -DRIVE_SPEED, DRIVE_SPEED));
                zeus.motorLeft.setPower(Range.clip(left, -DRIVE_SPEED, DRIVE_SPEED));
                break;
            case INVERTED:
                zeus.motorRight.setPower(Range.clip(-left, -DRIVE_SPEED, DRIVE_SPEED));
                zeus.motorLeft.setPower(Range.clip(-right, -DRIVE_SPEED, DRIVE_SPEED));
                break;
            default:
                break;
        }
    }

    //Driving
    private void controlCycle() {
        switch (currentControl) {
            case 0:
                currentControl += 1;
                control = controls[currentControl];
                break;
            case 1:
                currentControl -= 1;
                control = controls[currentControl];
                break;
            default:
                break;
        }
        wait.waitMilliseconds(250);
    }
    private void speedCycle() {
        switch (currentSpeed) {
            case 0:
                currentSpeed += 1;
                speed = speeds[currentSpeed];
                speedSet();
                break;
            case 1:
                currentSpeed -= 1;
                speed = speeds[currentSpeed];
                speedSet();
                break;
            default:
                break;
        }
        wait.waitMilliseconds(250);
    }
    private void speedSet() {
        switch (speed) {
            case DRIVE_FAST:
                DRIVE_SPEED = HardwareZeus.FAST_DRIVE_SPEED;
                break;
            case DRIVE_SLOW:
                DRIVE_SPEED = HardwareZeus.DRIVE_SPEED;
                break;
            default:
                break;
        }
    }

    //Claw
    private void clawSet(@NonNull Position position) {
        switch (position) {
            case CLAW_OPEN:
                zeus.servo1.setPosition(CLAW_OPEN);
                zeus.servo2.setPosition(CLAW_OPEN);
                wait.waitMilliseconds(250);
                break;
            case CLAW_CLOSED:
                zeus.servo1.setPosition(CLAW_CLOSED);
                zeus.servo2.setPosition(CLAW_CLOSED);
                wait.waitMilliseconds(250);
                break;
            default:
                break;
        }
    }
    private void clawCycle() {
        switch (currentPosition) {
            case 0:
                currentPosition += 1;
                clawSet(position[currentPosition]);
                break;
            case 1:
                currentPosition -= 1;
                clawSet(position[currentPosition]);
                break;
            default:
                break;
        }
    }

    private enum Position {
        CLAW_OPEN, CLAW_CLOSED
    }

    private enum Speed {
        DRIVE_FAST, DRIVE_SLOW
    }

    private enum Controls {
        NORMAL, INVERTED
    }


}
