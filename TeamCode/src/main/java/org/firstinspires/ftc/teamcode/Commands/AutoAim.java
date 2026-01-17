package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutoAim {

    private GoBildaPinpointDriver pinpoint;

    // Fixed turn settings
    private static final double TURN_POWER = 0.35;
    private static final double ANGLE_TOLERANCE_DEG = 1.5;

    public AutoAim(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "OdometryComputer");

        // Reset heading reference
        pinpoint.setHeading(0, AngleUnit.DEGREES);
    }

    public double getTurnPower(double targetYawDeg) {
        double currentYaw = pinpoint.getHeading(AngleUnit.DEGREES);

        double error = angleWrap(targetYawDeg - currentYaw);

        if (Math.abs(error) <= ANGLE_TOLERANCE_DEG) {
            return 0;
        }

        // Direction only, no scaling
        return error > 0 ? TURN_POWER : -TURN_POWER;
    }

    public boolean isAimed(double targetYawDeg) {
        double currentYaw = pinpoint.getHeading(AngleUnit.DEGREES);
        return Math.abs(angleWrap(targetYawDeg - currentYaw)) <= ANGLE_TOLERANCE_DEG;
    }

    private double angleWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void restGyro(double gyroResetAngle){
        pinpoint.setHeading(gyroResetAngle, AngleUnit.DEGREES);
    }
}