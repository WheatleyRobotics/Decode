package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.util.LimeLight;

public class LimeLightAutoAlign {

    private LimeLight limeLight;

    // tuning constant for turning
    private double kTurn = 0.02; //0.02

    // tolerance for alignment in degrees
    private double turnTolerance = 0.5;

    // optional: stop driving if too far away
    private double minDistance = 5; // inches, for safety

    public LimeLightAutoAlign(LimeLight limeLight) {
        this.limeLight = limeLight;
    }

    /**
     * Returns only turn power needed to align.
     * forward/back power is always 0 unless too far away.
     */
    public double update() {
        if (!limeLight.hasTarget()) {
            return 0;
        }

        double tx = limeLight.camera.getLatestResult().getTx(); // horizontal offset

        // ← Add a fixed left offset (positive turns robot left)
        double desiredOffset = 3.0; // degrees to the left
        tx -= desiredOffset;

        double turnPower = -tx * kTurn;
        turnPower = Range.clip(turnPower, -0.5, 0.5);

        if (Math.abs(tx) < turnTolerance) {
            turnPower = 0;
        }

        double distance = limeLight.distanceFromTagInches();
        if (distance < minDistance) {
            turnPower = 0;
        }

        return turnPower;
    }

    /** Returns true when robot is facing the target within tolerance */
    public boolean isAligned() {
        if (!limeLight.hasTarget()) return false;

        double tx = limeLight.camera.getLatestResult().getTx();
        return Math.abs(tx) < turnTolerance;
    }
}