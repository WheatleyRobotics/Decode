package org.firstinspires.ftc.teamcode.Subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Commands.AutoAim;

public class LimeLight{
    private Limelight3A limelight;

    //private AutoAim autoAim;

    public LimeLight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(8); //april tag pipeline #11 pipeline

        limelight.start();
    }

    public void update(){
        //limelight.updateRobotOrientation(autoAim.getYaw());
        LLResult llResult = limelight.getLatestResult();

        if(llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();

            telemetry.addData("Tx: ", llResult.getTx());
            telemetry.addData("Ty: ", llResult.getTy());
            telemetry.addData("Ta: ", llResult.getTa());
        }
    }
}
