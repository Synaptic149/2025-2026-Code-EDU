package Opmode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(group = "Tests", name = "mangomustard")

public class mangomustard extends OpMode {
    private Follower follower;

    public void init () {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
    }

    @Override
    public void loop(){
        follower.startTeleopDrive();
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
    }
}
