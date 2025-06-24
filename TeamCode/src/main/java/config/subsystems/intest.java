package config.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static config.core.robotConstants.*;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "Tests", name = "intake")
public class intest extends OpMode {
    Gamepad g1;

    private Telemetry telemetry;

    public DcMotor in;

    public void init() {
        in = hardwareMap.get(DcMotor.class, inmotorname);
        in.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        in .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) {
            in.setPower(0.5);
        } else if (gamepad1.dpad_down) {
            in.setPower(-0.5);
        } else if(gamepad1.triangle) {
            in.setPower(0);
        }
    }
}
