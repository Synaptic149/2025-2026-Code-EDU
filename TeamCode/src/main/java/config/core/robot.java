package config.core;


// pp !! ! ! ! ! ! !
import static config.core.robotConstants.robotCentric;
import static config.core.robotConstants.speed;
import static config.core.robotConstants.turnSpeed;

import com.pedropathing.follower.Follower;


// ftc robot stuff
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

// telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry;

import config.subsystems.extend;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import config.core.robotConstants.*;

public class robot {
    private HardwareMap h;
    private Telemetry t;
    private Gamepad d1,d2, currentG1, currentG2, prevG1, prevG2;
    private Follower f;
    private extend out;
    private Pose startPose;

    public robot(HardwareMap h, Telemetry t, Gamepad d1, Gamepad d2 ) {
        this.h = h; // from my understanding, h and t are telemetry and hardware map of the new Opmode that are passed into the method, therefore it is necessary to use them to init everything
        this.t = t;
        this.d1 = d1;
        this.d2 = d2;
        f = new Follower(this.h, FConstants.class, LConstants.class); // init follower
        out = new extend(this.h,this.t); // init the extend stuff
        this.currentG1 = new Gamepad(); // init Game-pads
        this.currentG2= new Gamepad();
        this.prevG1 = new Gamepad(); // init Game-pads
        this.prevG2= new Gamepad();

    }
    public void periodic() { // runs to update everything. should be in the opmode loop
        f.update();
        out.periodic();
        t.update();
    }
    public void dualControls() {
        prevG1.copy(currentG1);
        prevG2.copy(currentG2);
        currentG1.copy(d1);
        currentG2.copy(d2);

        f.setTeleOpMovementVectors(speed * -currentG1.left_stick_y, speed * -currentG1.left_stick_x, turnSpeed * -currentG1.right_stick_x, robotCentric);

        if (currentG1.triangle && !prevG1.triangle) {
            out.tofull();
        } else if (currentG1.cross && !prevG1.cross) {
            out.tozero();
        } else if (currentG1.dpad_up && !prevG1.dpad_up) {
        out.tohalf();
        } else if (currentG1.square && !prevG1.square) {
            out.tothird();
        } else if (currentG1.circle && !prevG1.circle) {
        out.totwothird();
    }
        t.addData("gamepad", currentG2.left_stick_y);


    }
    public void start() {
        f.startTeleopDrive();
    }


}
