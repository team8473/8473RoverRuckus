package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.commands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus;

public class Lift extends LinearOpMode {

    private HardwareZeus zeus = new HardwareZeus();

    @Override
    public void runOpMode() throws InterruptedException {
        zeus.init(hardwareMap);
    }

    public void lift () {
        float lift = gamepad2.left_stick_y;
        lift = Range.clip(-lift, -1, 1);
        zeus.motorLift.setPower(Range.clip(lift, -.6, .6));
    }
}
