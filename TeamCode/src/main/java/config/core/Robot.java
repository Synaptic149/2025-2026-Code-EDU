package config.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Robot extends LinearOpMode {
    public void initHardware() {
        DcMotor motorone = hardwareMap.get(DcMotor.class, "hi");
    }

}
