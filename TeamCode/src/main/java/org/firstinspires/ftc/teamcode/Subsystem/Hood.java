package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hood {
    private final CRServo hood;

    private static final double hoodPower = 1;

    public Hood(HardwareMap hardwareMap) {
        hood = hardwareMap.get(CRServo.class, "Hood");
        hood.setDirection(CRServo.Direction.FORWARD);
    }

    public void up(){
        hood.setPower(hoodPower);
    }

    public void down(){
        hood.setPower(-hoodPower);
    }

    public void stop(){
        hood.setPower(0);
    }
}