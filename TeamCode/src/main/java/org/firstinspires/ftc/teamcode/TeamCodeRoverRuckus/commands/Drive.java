package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.commands;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.DRIVE_SPEED;

@Disabled
public class Drive extends LinearOpMode {

    private HardwareZeus zeus = new HardwareZeus();

    @Override
    public void runOpMode() throws InterruptedException {
        zeus.init(hardwareMap);
    }

    public void drive() {
        float right = gamepad1.left_stick_y;
        float left = -gamepad1.right_stick_y;

        left = Range.clip(-left, -1, 1);
        right = Range.clip(-right, -1, 1);

        zeus.motorRight.setPower(Range.clip(right, -DRIVE_SPEED, DRIVE_SPEED));
        zeus.motorLeft.setPower(Range.clip(left, -DRIVE_SPEED, DRIVE_SPEED));
    }
}
