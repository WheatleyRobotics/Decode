package org.firstinspires.ftc.teamcode.Subsystem;

import com.pedropathing.control.KalmanFilter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class LimeLight {
    private Follower follower;
    private boolean following = false;
    private final Pose startingPose = new Pose(27.2, 132.283, Math.toRadians(143));

    private Limelight3A limelight;
    private LLResult llResult;

    public LimeLight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        limelight.start();
    }

    public void update() {
        llResult = limelight.getLatestResult();
        follower.update();

        follower.setPose(getPedroPose());

        if (following && !follower.isBusy()) following = false;
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

    /*
    private Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //TODO: Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this

        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
     */

    public Pose getPedroPose() {
        Pose3D pose = llResult.getBotpose_MT2();

        Pose ftcPose = new Pose(
                pose.getPosition().x,
                pose.getPosition().y,
                pose.getOrientation().getYaw(),
                FTCCoordinates.INSTANCE
        );

        return ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}