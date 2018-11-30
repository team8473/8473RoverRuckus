package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pixy.PixyCam_Analog;

@TeleOp(name = "TestAnalog", group = "test")
@Disabled
public class TestPixyAnalog extends OpMode {

    private PixyCam_Analog pixy = null;

    private Servo servo = null;

    @Override
    public void init() {
        pixy = new PixyCam_Analog(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Position", servo.getPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
    }
}
