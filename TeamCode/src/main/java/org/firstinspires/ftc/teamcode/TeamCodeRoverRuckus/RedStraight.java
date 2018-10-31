package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.commands.AutoDrive;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kD;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kI;
import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.kP;

@Autonomous(name = "Straight", group = "Red")
public class RedStraight extends LinearOpMode {

    private HardwareZeus zeus = new HardwareZeus();
    private ElapsedTime timer = new ElapsedTime();
    private AutoDrive auto = null;

    private Orientation angles = null;

    private double lastError, integralError;
    private double goalAngle;

    @Override
    public void runOpMode() throws InterruptedException {
        auto = new AutoDrive();

        zeus.init(hardwareMap);

        zeus.servo1.setPosition(CLAW_CLOSED);
        zeus.servo2.setPosition(CLAW_CLOSED);

        zeus.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zeus.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(10);
        zeus.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        zeus.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        angles = zeus.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        goalAngle = angles.firstAngle;

        auto.gyroDrive(DRIVE_SPEED, 55, goalAngle);
        zeus.servo1.setPosition(CLAW_OPEN);
        zeus.servo2.setPosition(CLAW_OPEN);
    }
}
