package Opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import config.core.robot;

@TeleOp(name = "Dual Drive", group = "Opmode")
public class DualDrive extends OpMode {

    robot r;

    @Override
    public void init() {
        r = new robot(hardwareMap, telemetry, gamepad1 , gamepad2);
    }

    @Override
    public void start() {
        r.start();
    }
    @Override
    public void loop() {
        r.dualControls();
        r.periodic();
    }
}