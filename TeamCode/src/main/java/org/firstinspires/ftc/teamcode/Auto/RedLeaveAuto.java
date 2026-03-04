package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystem.Hood;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Tele.TeleConstant;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Shoot And Leave Red")
public class RedLeaveAuto extends OpMode {
    private final AutoConstants autoConstants = new AutoConstants();

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Hood hood;

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
            new Pose(118.9603399433428, 130.38810198300283, Math.toRadians(37));

    private final Pose intakePose =
            new Pose(73.68271954674222, 132.60906515580734, Math.toRadians(37));

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

                hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.bumperUpOffset);
                shooter.setTargetRPM(TeleConstant.bumperUpRPM);

                if (pathTimer.getElapsedTimeSeconds() >
                        autoConstants.preLoadOnlySpinUpShooterWait) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:

                if (shooter.isAtTargetRPM()) {
                    shooter.setIndexerPower(autoConstants.indexerPower);
                    intake.intakeIn();
                }

                if (pathTimer.getElapsedTimeSeconds() >
                        autoConstants.shootPreLoadBalls) {

                    shooter.setIndexerPower(0);   // stop feeding
                    intake.stopIntaking();
                    shooter.autoShooter();        // stop shooter

                    setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS);
                }
                break;

            case DRIVE_STARTPOS_SHOOT_POS:

                follower.followPath(driveStartPoseShootPos, true);
                setPathState(PathState.DONE);
                break;

            case DONE:
                break;
        }
    }

    @Override
    public void init() {

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);

        buildPaths();
        follower.setPose(startingPose);

        pathState = PathState.SPIN_UP_SHOOTER;

        shooter.auto = true;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        // DO NOT change state here or it will skip preload
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
        telemetry.update();
    }
}