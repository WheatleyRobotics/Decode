package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    private final Servo hood;

    private static final double hoodPower = 1;

    private double manualVal = 0;

    public Hood(HardwareMap hardwareMap) {
        hood = hardwareMap.get(Servo.class, "Hood");
        hood.setDirection(Servo.Direction.FORWARD);
    }

    public void setHoodPos(double hoodPos){
        hood.setPosition(hoodPos);
    }

    public void manualDown(){
        manualVal += 0.1;
        hood.setPosition(manualVal);
    }

    public void manualUp(){
        manualVal -= 0.1;
        hood.setPosition(manualVal);
    }

    public double getServoPos(){
        return hood.getPosition();
    }

    public void resetServo(){
        hood.resetDeviceConfigurationForOpMode();
    }

    public void setRange(double min, double max){
        hood.scaleRange(min, max);
    }

    /*
    public void stop(){
        hood.setPower(0);
    }
     */
}