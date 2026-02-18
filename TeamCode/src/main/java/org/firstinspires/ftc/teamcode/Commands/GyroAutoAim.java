package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class GyroAutoAim {

    private GoBildaPinpointDriver pinpoint;

    // ---- TUNING VALUES ----
    private static final double KP = 0.012;
    private static final double MAX_TURN_POWER = 0.45;
    private static final double ANGLE_TOLERANCE_DEG = 0.5; //1.0

    public GyroAutoAim(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "OdometryComputer");
    }

    public double getTurnPower(double targetYawDeg) {
        double currentYaw = getYaw();

        double error = angleWrap(targetYawDeg - currentYaw);

        if (Math.abs(error) <= ANGLE_TOLERANCE_DEG) {
            return 0;
        }

        double turnPower = error * KP;

        if (turnPower > MAX_TURN_POWER) {
            turnPower = MAX_TURN_POWER;
        }
        if (turnPower < -MAX_TURN_POWER) {
            turnPower = -MAX_TURN_POWER;
        }

        return turnPower;
    }

    public boolean isAimed(double targetYawDeg) {
        double error = angleWrap(targetYawDeg - getYaw());
        return Math.abs(error) <= ANGLE_TOLERANCE_DEG;
    }

    public double getYaw() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    private double angleWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void resetGyro(double newGyroPos) {
        pinpoint.setHeading(newGyroPos, AngleUnit.DEGREES);
    }
}