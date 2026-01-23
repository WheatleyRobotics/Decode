package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimeLight {
    private Limelight3A limelight;
    private LLResult latestResult;

    public LimeLight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(8);
        limelight.start();
    }

    public void update() {
        latestResult = limelight.getLatestResult();
    }

    public boolean hasValidTarget() {
        return latestResult != null && latestResult.isValid();
    }

    public double getTx() {
        if (hasValidTarget()) {
            return latestResult.getTx();
        }
        return 0.0;
    }

    public double getTy() {
        if (hasValidTarget()) {
            return latestResult.getTy();
        }
        return 0.0;
    }

    public double getTa() {
        if (hasValidTarget()) {
            return latestResult.getTa();
        }
        return 0.0;
    }
}
