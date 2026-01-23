package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimeLight {
    private Limelight3A limelight;
    private LLResult llResult;

    public LimeLight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(8);
        limelight.start();
    }

    public void update() {
        llResult = limelight.getLatestResult();
    }

    public boolean hasValidTarget() {
        return llResult != null && llResult.isValid();
    }

    public double getTx() {
        if (hasValidTarget()) {
            return llResult.getTx();
        }
        return 0.0;
    }

    public double getTy() {
        if (hasValidTarget()) {
            return llResult.getTy();
        }
        return 0.0;
    }

    public double getTa() {
        if (hasValidTarget()) {
            return llResult.getTa();
        }
        return 0.0;
    }

    public Pose3D getBotPose(){
        Pose3D botPose = llResult.getBotpose_MT2();
        return botPose;
    }
}
