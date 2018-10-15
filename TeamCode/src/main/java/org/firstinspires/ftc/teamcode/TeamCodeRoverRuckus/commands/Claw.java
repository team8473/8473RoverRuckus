package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.commands;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus;

public class Claw extends LinearOpMode{

    private HardwareZeus zeus = new HardwareZeus();
    private Wait wait = new Wait();

    private int currentPosition = 1;

    private static Position[] position = new Position[] {Position.CLAW_CLOSED, Position.CLAW_OPEN};

    @Override
    public void runOpMode() throws InterruptedException {
        zeus.init(hardwareMap);
    }

    private void clawSet (Position position) {
        switch (position) {
            case CLAW_OPEN:
                wait.waitMilliseconds(250);
                break;
            case CLAW_CLOSED:
                wait.waitMilliseconds(250);
                break;
            default:
                break;
        }
    }

    public void clawCycle () {
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
}
