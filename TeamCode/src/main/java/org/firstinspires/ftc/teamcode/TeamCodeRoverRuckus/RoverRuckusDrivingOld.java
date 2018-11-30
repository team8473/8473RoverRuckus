package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus;
import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.commands.Wait;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.FAST_DRIVE_SPEED;

@TeleOp(name = "Driving")
@Disabled
public class RoverRuckusDrivingOld extends OpMode {

    private HardwareZeus zeus = new HardwareZeus();
    private Wait wait = new Wait();

    private int currentPosition = 1;
    private int currentSpeed = 1;
    private Speed speed;

    private static Position[] position = new Position[] {Position.CLAW_CLOSED, Position.CLAW_OPEN};
    private static Speed[] speeds = new Speed[] {Speed.DRIVE_FAST, Speed.DRIVE_SLOW};

    @Override
    public void init() {
        zeus.init(hardwareMap);
        speed = speeds[currentSpeed];
        telemetry.addData("Yo! What up peeps", "Press that triangle down there to start playing man");
        telemetry.update();
    }

    @Override
    public void start() {
        zeus.runtime.reset();
    }

    @Override
    public void loop() {

        //Claw
        if (gamepad2.a) {
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

        //Lift
        float lift = gamepad2.left_stick_y;
        lift = Range.clip(-lift, -1, 1);
        zeus.motorLift.setPower(Range.clip(lift, -.6, .6));

        if (gamepad2.y) {
            zeus.motorLift.setPower(1.0);
        } else if (gamepad2.left_stick_y == 0.0){
            zeus.motorLift.setPower(0);
        }

        //Driving
        float right = -gamepad1.right_stick_y;
        float left = -gamepad1.left_stick_y;

        left = Range.clip(-left, -1, 1);
        right = Range.clip(-right, -1, 1);

        if (gamepad1.right_bumper) {
            switch (currentSpeed) {
                case 0:
                    currentSpeed += 1;
                    speed = speeds[currentSpeed];
                    break;
                case 1:
                    currentSpeed -= 1;
                    speed = speeds[currentSpeed];
                    break;
                default:
                    break;
            }
            wait.waitMilliseconds(250);
        }

        switch (speed) {
            case DRIVE_FAST:
                zeus.motorRight.setPower(Range.clip(right, -FAST_DRIVE_SPEED, FAST_DRIVE_SPEED));
                zeus.motorLeft.setPower(Range.clip(left, -FAST_DRIVE_SPEED, FAST_DRIVE_SPEED));
                break;
            case DRIVE_SLOW:
                zeus.motorRight.setPower(Range.clip(right, -DRIVE_SPEED, DRIVE_SPEED));
                zeus.motorLeft.setPower(Range.clip(left, -DRIVE_SPEED, DRIVE_SPEED));
                break;
            default:
                break;
        }
    }

    private void clawSet (@NonNull Position position) {
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

    private enum Position {
        CLAW_OPEN, CLAW_CLOSED
    }

    private enum Speed {
        DRIVE_FAST, DRIVE_SLOW
    }
}
