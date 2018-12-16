package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
public class HardwareZeus {

    //Motors
    public DcMotor motorRight = null;
    public DcMotor motorLeft = null;
    public DcMotor motorLift = null;
    public DcMotor jesus = null;

    //Servos
    public CRServo george = null;
    public Servo servo1 = null;
    public Servo servo2 = null;

    //Sensors
    public BNO055IMU imu = null;

    //Encoder Variables
    private static final double COUNTS_PER_MOTOR_REV = 350.0;
    private static final double DRIVE_GEAR_REDUCTION = 2.0;
    private static final double WHEEL_DIAMETER = 4.0;
    private static final double ROBOT_WIDTH = 14.0;
    private static final double ROBOT_LENGTH = 15.0;
    private static final double SHORT_SQUARED = Math.pow(0.5 * ROBOT_WIDTH, 2.0);
    private static final double LONG_SQUARED = Math.pow(0.5 * ROBOT_LENGTH, 2.0);
    private static final double SQUARE_ROOT = Math.sqrt(2.0 * (SHORT_SQUARED + LONG_SQUARED));
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER * 3.1415);
    public static final double DRIVE_SPEED = 0.5;
    public static final double FAST_DRIVE_SPEED = 1.0;
    public static final double SLOW_DRIVE_SPEED = 0.4;
    public static final double TURN_SPEED = 0.45;
    public static final double ROBOT_CIRCUMFERENCE = (Math.PI * SQUARE_ROOT);
    public ElapsedTime runtime = new ElapsedTime();

    //Pixy PID Loop Variables
    public static final double kP = 0.03;
    public static final double kI = 0.0;
    public static final double kD = 0.06;

    //Claw
    public static final double CLAW_OPEN = 0.5;
    public static final double CLAW_CLOSED = 1.0;

    //George
    public static final double GEORGE_OFF = -0.085;

    //Strings
    public static final String RIGHT = "RIGHT";
    public static final String LEFT  = "LEFT";

    private HardwareMap hwMap = null;

    public HardwareZeus(){
    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        //Motors
        motorRight = hwMap.dcMotor.get("right");
        motorLeft = hwMap.dcMotor.get("left");
        motorLift = hwMap.dcMotor.get("lift");
        jesus = hwMap.dcMotor.get("jesus");

        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);

        motorRight.setPower(0.0);
        motorLeft.setPower(0.0);
        motorLift.setPower(0.0);
        jesus.setPower(0.0);

        //Servo
        george = hwMap.crservo.get("george");
        servo1 = hwMap.servo.get("servo1");
        servo2 = hwMap.servo.get("servo2");

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);

        //Sensors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}
