package Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import config.subsystems.extend;

@Config
@TeleOp(group = "Tests", name = "Lift Test")
public class outtest extends OpMode {
    public static int target = 0;

    extend e;
    Gamepad g1, p1;

    @Override
    public void init() {
        e = new extend(hardwareMap, telemetry);

        g1 = new Gamepad();
        p1 = new Gamepad();

    }

    @Override
    public void loop() {
        p1.copy(g1);
        g1.copy(gamepad1);

        if (g1.triangle) {
            e.tofull();
        } else if (g1.cross) {
            e.tozero();
        } else if (g1.square) {
            e.tothird();
        } else if(g1.circle) {
            e.totwothird();

        } else if(g1.dpad_up) {
            e.tohalf();
        }
        e.periodic();
        telemetry.update();
    }
}
