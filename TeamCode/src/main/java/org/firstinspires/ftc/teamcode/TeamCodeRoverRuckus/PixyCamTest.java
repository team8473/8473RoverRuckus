package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Pixy Cam Test", group = "Test")
public class PixyCamTest extends LinearOpMode {
    private Servo pixyServo = null;
    private AnalogInput analogIn_pixy = null;

    @Override
    public void runOpMode() throws InterruptedException {
        analogIn_pixy = hardwareMap.analogInput.get("pixy_analog");       // if it doesn't work, try the above

        pixyServo = hardwareMap.servo.get("pixyServo");

        double pixyStart = pixyServo.getPosition();

        telemetry.addData("Servo position = ", pixyServo.getPosition());
        telemetry.update();

        waitForStart();

        pixyServo.setPosition(0.5);
        sleep(5000);
        pixyServo.setPosition(0.0);
        sleep(5000);
        pixyServo.setPosition(0.5);
        sleep(5000);
    }

    public double getAnalogRead() {   // from 0 to 3.3V where 1.15V=middle, higher means object is to the right
        return  analogIn_pixy.getVoltage();
    }
}
