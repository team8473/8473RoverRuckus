package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pixy.PixyCam_Analog;

@TeleOp(name = "TestAnalog", group = "test")
public class TestPixyAnalog extends OpMode {

    private PixyCam_Analog pixy = null;

    @Override
    public void init() {
        pixy = new PixyCam_Analog(hardwareMap);
    }

    @Override
    public void init_loop() {
        double block = pixy.getAnalogRead();
        if (block > 1.15) {
            telemetry.addData("left", block);
            telemetry.update();
        } else if (block < 1.15) {
            telemetry.addData("right", block);
            telemetry.update();
        }
    }

    @Override
    public void loop() {
    }
}
