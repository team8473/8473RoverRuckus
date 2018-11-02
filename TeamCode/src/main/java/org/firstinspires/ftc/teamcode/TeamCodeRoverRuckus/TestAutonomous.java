package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.commands.AutoDrive;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.RIGHT;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.TURN_SPEED;

@Autonomous (name = "Test", group = "Main")
@Disabled
public class TestAutonomous extends LinearOpMode {

    private HardwareZeus zeus = new HardwareZeus();
    private ElapsedTime timer = new ElapsedTime();
    private AutoDrive auto = new AutoDrive();

    private double lastError, integralError;
    private double goalAngle, xGoal;

    private Orientation angles = null;

    @Override
    public void runOpMode() throws InterruptedException {

        zeus.init(hardwareMap);

        angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        goalAngle = angles.firstAngle;

        zeus.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zeus.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(10);
        zeus.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        zeus.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        timer.reset();

        while(timer.seconds() > 2.5){
            zeus.motorLift.setPower(-0.5);
        }
        zeus.motorLift.setPower(0.0);

        auto.encoderTurn(180, RIGHT);
        auto.pixyDrive(DRIVE_SPEED, -20);

        if (angles.firstAngle < -5) {
//            gyroDrive(DRIVE_SPEED, 10, Whatever angle it needs to be);
            zeus.servo1.setPosition(.5);
            zeus.servo2.setPosition(.5);
        } else if (angles.firstAngle > 5) {
//            gyroDrive(DRIVE_SPEED, 10, Whatever angle it needs to be);
            zeus.servo1.setPosition(.5);
            zeus.servo2.setPosition(.5);
        } else {
            auto.gyroDrive(DRIVE_SPEED, 10, goalAngle);
            zeus.servo1.setPosition(.5);
            zeus.servo2.setPosition(.5);
        }
    }
}