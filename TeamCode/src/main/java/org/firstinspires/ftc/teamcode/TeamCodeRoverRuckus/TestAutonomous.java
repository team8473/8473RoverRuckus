package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.commands.AutoDrive;

import java.util.Scanner;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.LEFT;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.RIGHT;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.ROBOT_CIRCUMFERENCE;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kD;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kI;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kP;

@Autonomous (name = "Test", group = "Main")
//@Disabled
public class TestAutonomous extends AutoDrive {

    private HardwareZeus zeus = new HardwareZeus();
    private ElapsedTime timer = new ElapsedTime();
    private AutoDrive auto = new AutoDrive();

    Scanner input = new Scanner(System.in);

    private double lastError, integralError;
    private double goalAngle, xGoal;

    private Orientation angles = null;
    private Acceleration gravity = null;


    @Override
    public void runOpMode() throws InterruptedException {

        zeus.init(hardwareMap);

        angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = zeus.imu.getGravity();
        goalAngle = angles.firstAngle;


        zeus.motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        zeus.motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        System.out.println("Enter Number : ");
        int number = input.nextInt();

        telemetry.addData("You Entered : ", number);
        telemetry.update();

        waitForStart();

        gravity = zeus.imu.getGravity();

        zeus.motorLift.setPower(-0.4);
        timer.reset();
        while (gravity.yAccel > -9.8 && timer.seconds() < 5.0) {
            gravity = zeus.imu.getGravity();
            telemetry.addData("Hi", gravity.yAccel);
            telemetry.update();
        }
        sleep(500);
        zeus.motorLift.setPower(0.0);

        zeus.motorLeft.setPower(-DRIVE_SPEED);
        zeus.motorRight.setPower(DRIVE_SPEED);
        timer.reset();
        while (timer.seconds() < 1.9) {
            telemetry.addData("Alison", "Sucks");
            telemetry.update();
        }
        zeus.motorRight.setPower(0.0);
        zeus.motorLeft.setPower(0.0);

        sleep(1000);

        zeus.motorRight.setPower(-DRIVE_SPEED);
        zeus.motorLeft.setPower(-DRIVE_SPEED);
        timer.reset();
        while (timer.seconds() < 1.0){
            telemetry.addData("Gabe","Is Awesome");
            telemetry.update();
        }
        zeus.motorLeft.setPower(0.0);
        zeus.motorRight.setPower(0.0);
    }
}