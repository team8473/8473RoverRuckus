import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus;

import static org.firstinspires.ftc.teamcode.TeamCodeRoverRuckus.HardwareZeus.DRIVE_SPEED;

@TeleOp(name = "Alison", group = "Test")
public class POVDrivingTest extends OpMode {
    private HardwareZeus zeus = new HardwareZeus();
    @Override
    public void init() {
        zeus.init(hardwareMap);
    }

    @Override
    public void loop() {
        float right = -gamepad1.right_stick_y;
        float left = -gamepad1.left_stick_x;

        left = Range.clip(-left, -1, 1);
        right = Range.clip(-right, -1, 1);

        if (gamepad1.right_stick_y > 0.0 || gamepad1.right_stick_y < 0.0) {
            zeus.motorRight.setPower(Range.clip(right, -DRIVE_SPEED, DRIVE_SPEED));
            zeus.motorLeft.setPower(Range.clip(right, -DRIVE_SPEED, DRIVE_SPEED));
        } else if (gamepad1.left_stick_x > 0.0 || gamepad1.left_stick_x < 0.0) {
            zeus.motorRight.setPower(Range.clip(-left, -DRIVE_SPEED, DRIVE_SPEED));
            zeus.motorLeft.setPower(Range.clip(left, -DRIVE_SPEED, DRIVE_SPEED));
        } else {
            zeus.motorRight.setPower(0.0);
            zeus.motorLeft.setPower(0.0);
        }
    }
}
