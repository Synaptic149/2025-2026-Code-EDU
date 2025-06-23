package config.core;

import static config.core.robotConstants.*;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import config.subsystems.extend;
import config.subsystems.intake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad currentG1,currentG2, prevG1, prevG2;
    private Follower follower;
    private extend extend;
    private intake intake;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad currentG1, Gamepad currentG2) { // robot
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        follower = new Follower(this.hardwareMap, FConstants.class, LConstants.class);
        extend = new extend(hardwareMap, telemetry);
        intake = new intake(hardwareMap, telemetry);
        this.currentG1 = new Gamepad();
        this.currentG2 = new Gamepad();



    }
    public void solodrive() {
        prevG1.copy(currentG1);
        prevG2.copy(currentG2);
        follower.setTeleOpMovementVectors(-currentG1.left_stick_y * speed,  speed * -currentG1.left_stick_x , speed * -currentG1.right_stick_x);
    }
}