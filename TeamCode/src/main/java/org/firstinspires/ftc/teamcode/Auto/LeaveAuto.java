package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Subsystem.Hood;

@Autonomous(name = "Red Leave Auto")
public class LeaveAuto extends OpMode {
    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    //private Hood hood;

    private Timer pathTimer;
    private Timer opModeTimer;

    public enum PathState {
        SPIN_UP_SHOOTER,
        SHOOT_PRELOAD,
        DRIVE_STARTPOS_SHOOT_POS,
        DONE
    }

    private PathState pathState;

    private final Pose startingPose =
            new Pose(64.19692307692308, 7.778461538461562, Math.toRadians(90));

    private final Pose intakePose =
            new Pose(38.27692307692308, 34.227692307692294, Math.toRadians(180));

    private PathChain driveStartPoseShootPos;

    public void buildPaths() {
        driveStartPoseShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, intakePose))
                .setLinearHeadingInterpolation(
                        startingPose.getHeading(),
                        intakePose.getHeading()
                )
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case SPIN_UP_SHOOTER:
                //hood.setHoodPos(0.6);
                shooter.setTargetRPM(130);
                if (shooter.isAtTargetRPM()
                        || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                shooter.setIndexerPower(1);
                intake.intakeIn();

                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    shooter.setIndexerPower(0);
                    setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS);
                }
                break;

            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPoseShootPos, true);
                setPathState(PathState.DONE);
                break;

            case DONE:
                // Do nothing
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        //shooter = new Shooter(hardwareMap);
        //intake = new Intake(hardwareMap);
        //hood = new Hood(hardwareMap);

        buildPaths();
        follower.setPose(startingPose);

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS);
        shooter.auto = true;
    }

    @Override
    public void loop() {
        shooter.update();
        follower.update();
        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Shooter RPM", shooter.getCurrentRPM());
        telemetry.addData("Target RPM", shooter.getTargetRPM());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}