package Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "motor test", group = "test")

public class motortest extends OpMode {
    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y > 0.1) {
            motor.setPower(gamepad1.left_stick_y);
        } else {
            motor.setPower(gamepad1.left_stick_y);

        }
        telemetry.addData("motor power", motor.getPower());
        telemetry.addData("motor ", motor.getConnectionInfo());
    }
}