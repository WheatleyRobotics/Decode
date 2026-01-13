package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

public class FirstAuto {
    private Follower follower;

    private Timer pathTimer, opModeTimer;

    public enum PathState{
        //Start POSITION_END POSITION
        //DRIVE>MOVEMENT STATE
        //SHOOT>ATTEMPT TO SCORE THE ARTIFACT

        DRIVE_STARTPOS_SHOOT_POS,

        SHOOT_PRELOAD
    }

    PathState pathState;

    private final Pose startingPose = new Pose(20.996923076923085, 121.64923076923078, Math.toRadians(324));
    private final Pose shootPose = new Pose(58.43692307692309, 84.73846153846152, Math.toRadians(130));

    private PathChain driveStartPoseShootPos;

    public void buildPaths(){
        //put in coordinates for starting pose > ending pose
        driveStartPoseShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, shootPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPoseShootPos, true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                //check if follower is done its path
                if(!follower.isBusy()) {
                    //TODO add logic to flywheel shooter
                    telemetry.addLine("Done Path 1");
                }
            default:
                telemetry.addLine("No State Command");
                break;

        }
    }

    @Override
    public void init(){

    }

    @Override
    public void loop(){

    }
}
