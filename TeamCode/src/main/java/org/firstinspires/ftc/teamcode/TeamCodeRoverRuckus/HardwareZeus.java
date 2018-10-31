package org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class HardwareZeus {

    //Motors
    public DcMotor motorRight = null;
    public DcMotor motorLeft = null;
    public DcMotor motorLift = null;

    //Servos
    public Servo servo1 = null;
    public Servo servo2 = null;

    //Sensors
    public PixyCam pixy = null;
    public BNO055IMU imu = null;

    //Encoder Variables
    private static final double COUNTS_PER_MOTOR_REV = 350.0;
    private static final double DRIVE_GEAR_REDUCTION = 2.0;
    private static final double WHEEL_DIAMETER = 4.0;
    private static final double ROBOT_SHORT_DIAMETER_IN = 16.35;
    private static final double ROBOT_LONG_DIAMETER_IN = 17.4;
    private static final double SHORT_SQUARED = Math.pow(0.5 * ROBOT_SHORT_DIAMETER_IN, 2.0);
    private static final double LONG_SQUARED = Math.pow(0.5 * ROBOT_LONG_DIAMETER_IN, 2.0);
    private static final double SQUARE_ROOT = Math.sqrt(2.0 * (SHORT_SQUARED + LONG_SQUARED));
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER * 3.1415);
    public static final double DRIVE_SPEED = 0.4;
    public static final double FAST_DRIVE_SPEED = 1.0;
    public static final double SLOW_DRIVE_SPEED = 0.4;
    public static final double TURN_SPEED = 0.2;
    public static final double ROBOT_CIRCUMFERENCE = (Math.PI * SQUARE_ROOT);
    public ElapsedTime runtime = new ElapsedTime();

    //Pixy PID Loop Variables
    public static final double kP = 0.03;
    public static final double kI = 0.0;
    public static final double kD = 0.06;

    //Claw
    public static final double CLAW_OPEN = 0.5;
    public static final double CLAW_CLOSED = 0.75;

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

        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorRight.setPower(0);
        motorLeft.setPower(0);
        motorLift.setPower(0);

        //Servo
        servo1 = hwMap.servo.get("servo1");
        servo2 = hwMap.servo.get("servo2");

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.FORWARD);

        //Sensors
//        pixy = hwMap.get(PixyCam.class, "pixy");

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
