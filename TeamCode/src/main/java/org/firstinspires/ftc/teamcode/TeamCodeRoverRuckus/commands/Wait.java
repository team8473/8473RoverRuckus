package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.commands;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Command to wait for a specified amount of time
 */
@Disabled
public class Wait extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
    }

    private ElapsedTime timer = new ElapsedTime();

    public void waitMilliseconds(int time) {
        timer.reset();
        while(timer.milliseconds() <= time) {
            telemetry.addData("Time : ", timer.milliseconds());
        }
    }

    public void waitSeconds(int time) {
        timer.reset();
        while (timer.seconds() <= time)
            telemetry.addData("Time : ", timer.seconds());
    }
}
