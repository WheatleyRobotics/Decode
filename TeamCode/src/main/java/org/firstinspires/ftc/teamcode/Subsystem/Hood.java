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
        manualVal = hoodPos;
    }

    public void manualUp() {
        manualVal += 0.01;
        manualVal = Math.max(0.0, Math.min(1.0, manualVal));
        hood.setPosition(manualVal);
    }

    public void manualDown() {
        manualVal -= 0.01;
        manualVal = Math.max(0.0, Math.min(1.0, manualVal));
        hood.setPosition(manualVal);
    }

    public double hoodPosVision(double distance) {
        /*
        double hoodPosVision = (3.04322e-7f) * (distance * distance * distance)
                - 0.0000976796f * (distance * distance)
                + 0.0100149f * distance
                + 0.35f;
         */

        double hoodPosVision = -0.0000424225 * Math.pow(distance, 2) + 0.00784549 * distance + 0.351313;
        return hoodPosVision;
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
}