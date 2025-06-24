package config.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static config.core.robotConstants.*;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "Tests", name = "spinny")

public class spinny extends OpMode{
    public DcMotor spinny;

    public void init () {
        spinny = hardwareMap.get(DcMotor.class, spinnymotorname);
        spinny.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinny.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            spinny.setPower(1);
        }
        else if (gamepad1.left_bumper) {
            spinny.setPower(-1);
    }   else {
            spinny.setPower(0);
        }
    }
}
