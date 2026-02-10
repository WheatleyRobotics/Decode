package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystem.Hood;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Tele.TeleConstant;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto With Constants ")
public class BlueAutoWAutoConstants extends OpMode {
    private final AutoConstants autoConstants = new AutoConstants();
    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Hood hood;

    private Timer pathTimer;
    private Timer shootDelayTimer;

    public enum PathState {
        SPIN_UP_SHOOTER,
        SHOOT_PRELOAD,

        GET_READY_TO_INTAKE_FIRST_BALLS,
        INTAKE_FIRST_BALLS,
        DRIVE_TO_SHOOT_FIRST_BALLS,
        SHOOT_FIRST_BALLS,

        GET_READY_TO_INTAKE_SECOND_BALLS,
        INTAKE_SECOND_BALLS,
        DRIVE_TO_SHOOT_SECOND_BALLS,
        SHOOT_SECOND_BALLS,

        GET_READY_TO_INTAKE_THIRD_BALLS,
        INTAKE_THIRD_BALLS,
        DRIVE_TO_SHOOT_THIRD_BALLS,
        SHOOT_THIRD_BALLS,

        ENDPOS,
        DONE
    }

    private PathState pathState;
    //private final int intakeBallsGyroPos = 181;

    // ===== POSES (UNCHANGED) =====
    private final Pose startingPose = new Pose(27.2, 132.283, Math.toRadians(143));
    private final Pose bumperUpPose = new Pose(27.2, 140.283, Math.toRadians(137));
    private final Pose firstIntakePose = new Pose(45.8215, 84.7384, Math.toRadians(autoConstants.intakeBallsGyroPos));
    private final Pose intakeFirstBallsPose = new Pose(30, 84.7323, Math.toRadians(autoConstants.intakeBallsGyroPos));
    private final Pose secondIntakePose = new Pose(45.8668, 61.4475, Math.toRadians(autoConstants.intakeBallsGyroPos));
    private final Pose intakeSecondBallsPose = new Pose(30, 59.5580, Math.toRadians(autoConstants.intakeBallsGyroPos));
    private final Pose thirdIntakePose = new Pose(45.8668, 35.6317, Math.toRadians(autoConstants.intakeBallsGyroPos));
    private final Pose intakeThirdBallsPose = new Pose(30, 35.6317, Math.toRadians(autoConstants.intakeBallsGyroPos));
    private final Pose endingPoint = new Pose(20, 56.8583, Math.toRadians(autoConstants.intakeBallsGyroPos));

    private PathChain setUpIntakeFirstBallsPos, intakeFirstBallsPos, shootFirstBallsPos,
            setUpIntakeSecondBallsPos, intakeSecondBallsPos, shootSecondBallsPos,
            setUpIntakeThirdBallsPos, intakeThirdBallsPos, shootThirdBallsPos,
            end;

    public void buildPaths() {
        setUpIntakeFirstBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, firstIntakePose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), firstIntakePose.getHeading())
                .build();

        intakeFirstBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(firstIntakePose, intakeFirstBallsPose))
                .setLinearHeadingInterpolation(firstIntakePose.getHeading(), intakeFirstBallsPose.getHeading())
                .build();

        shootFirstBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeFirstBallsPose, startingPose))
                .setLinearHeadingInterpolation(intakeFirstBallsPose.getHeading(), bumperUpPose.getHeading())
                .build();

        setUpIntakeSecondBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, secondIntakePose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), secondIntakePose.getHeading())
                .build();

        intakeSecondBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(secondIntakePose, intakeSecondBallsPose))
                .setLinearHeadingInterpolation(secondIntakePose.getHeading(), intakeSecondBallsPose.getHeading())
                .build();

        shootSecondBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeSecondBallsPose, startingPose))
                .setLinearHeadingInterpolation(intakeSecondBallsPose.getHeading(), bumperUpPose.getHeading())
                .build();

        setUpIntakeThirdBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, thirdIntakePose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), thirdIntakePose.getHeading())
                .build();

        intakeThirdBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(thirdIntakePose, intakeThirdBallsPose))
                .setLinearHeadingInterpolation(thirdIntakePose.getHeading(), intakeThirdBallsPose.getHeading())
                .build();

        shootThirdBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeThirdBallsPose, startingPose))
                .setLinearHeadingInterpolation(intakeThirdBallsPose.getHeading(), bumperUpPose.getHeading())
                .build();

        end = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, endingPoint))
                .setLinearHeadingInterpolation(startingPose.getHeading(), endingPoint.getHeading())
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        if (newState.toString().startsWith("SHOOT_")) {
            shootDelayTimer = new Timer(); // reset shoot delay when entering shoot state
        }
    }

    /*
    private final double timeToShoot = 6;
    private final double preLoadWait = 1.8;
     */

    public void statePathUpdate() {
        switch (pathState) {
            case SPIN_UP_SHOOTER:
                //hood.setHoodPos(0.6);
                shooter.setTargetRPM(TeleConstant.bumperUpRPM);
                if (pathTimer.getElapsedTimeSeconds() > autoConstants.spinUpShooterWait) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                shooter.setIndexerPower(autoConstants.indexerPower);
                intake.intakeIn();
                if (pathTimer.getElapsedTimeSeconds() > autoConstants.shootFirstBalls) {
                    shooter.stopShooter();
                    intake.stopIntaking();
                    setPathState(PathState.GET_READY_TO_INTAKE_FIRST_BALLS);
                }
                break;

            // ===== FIRST BALLS =====
            case GET_READY_TO_INTAKE_FIRST_BALLS:
                follower.followPath(setUpIntakeFirstBallsPos, true);
                setPathState(PathState.INTAKE_FIRST_BALLS);
                break;

            case INTAKE_FIRST_BALLS:
                intake.intakeIn();
                if (!follower.isBusy()) {
                    follower.followPath(intakeFirstBallsPos, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_FIRST_BALLS);
                }
                break;

            case DRIVE_TO_SHOOT_FIRST_BALLS:
                if (!follower.isBusy()) {
                    intake.stopIntaking();
                    follower.followPath(shootFirstBallsPos, true);
                    setPathState(PathState.SHOOT_FIRST_BALLS);
                }
                break;

            case SHOOT_FIRST_BALLS:
                if (!follower.isBusy() && shootDelayTimer.getElapsedTimeSeconds() > autoConstants.shootDelayTimer) {
                    shooter.setTargetRPM(TeleConstant.bumperUpRPM);
                    intake.intakeIn();
                    shooter.setIndexerPower(autoConstants.indexerPower);
                    if (pathTimer.getElapsedTimeSeconds() > autoConstants.timeToShoot) {
                        shooter.stopShooter();
                        intake.stopIntaking();
                        setPathState(PathState.GET_READY_TO_INTAKE_SECOND_BALLS);
                    }
                }
                break;

            // ===== SECOND BALLS =====
            case GET_READY_TO_INTAKE_SECOND_BALLS:
                follower.followPath(setUpIntakeSecondBallsPos, true);
                setPathState(PathState.INTAKE_SECOND_BALLS);
                break;

            case INTAKE_SECOND_BALLS:
                intake.intakeIn();
                if (!follower.isBusy()) {
                    follower.followPath(intakeSecondBallsPos, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_SECOND_BALLS);
                }
                break;

            case DRIVE_TO_SHOOT_SECOND_BALLS:
                if (!follower.isBusy()) {
                    intake.stopIntaking();
                    follower.followPath(shootSecondBallsPos, true);
                    setPathState(PathState.SHOOT_SECOND_BALLS);
                }
                break;

            case SHOOT_SECOND_BALLS:
                if (!follower.isBusy() && shootDelayTimer.getElapsedTimeSeconds() > autoConstants.shootDelayTimer) {
                    shooter.setTargetRPM(TeleConstant.bumperUpRPM);
                    intake.intakeIn();
                    shooter.setIndexerPower(autoConstants.indexerPower);
                    if (pathTimer.getElapsedTimeSeconds() > autoConstants.timeToShoot) {
                        shooter.stopShooter();
                        intake.stopIntaking();
                        setPathState(PathState.GET_READY_TO_INTAKE_THIRD_BALLS);
                    }
                }
                break;

            // ===== THIRD BALLS =====
            case GET_READY_TO_INTAKE_THIRD_BALLS:
                follower.followPath(setUpIntakeThirdBallsPos, true);
                setPathState(PathState.INTAKE_THIRD_BALLS);
                break;

            case INTAKE_THIRD_BALLS:
                intake.intakeIn();
                if (!follower.isBusy()) {
                    follower.followPath(intakeThirdBallsPos, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_THIRD_BALLS);
                }
                break;

            case DRIVE_TO_SHOOT_THIRD_BALLS:
                if (!follower.isBusy()) {
                    intake.stopIntaking();
                    follower.followPath(shootThirdBallsPos, true);
                    setPathState(PathState.SHOOT_THIRD_BALLS);
                }
                break;

            case SHOOT_THIRD_BALLS:
                if (!follower.isBusy() && shootDelayTimer.getElapsedTimeSeconds() > autoConstants.shootDelayTimer) {
                    shooter.setTargetRPM(TeleConstant.bumperUpRPM);
                    intake.intakeIn();
                    shooter.setIndexerPower(autoConstants.indexerPower);
                    if (pathTimer.getElapsedTimeSeconds() > autoConstants.timeToShoot) {
                        shooter.stopShooter();
                        intake.stopIntaking();
                        setPathState(PathState.ENDPOS);
                    }
                }
                break;

            case ENDPOS:
                //hood.setHoodPos(0);
                follower.followPath(end, true);
                setPathState(PathState.DONE);
                break;

            case DONE:
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        //hood = new Hood(hardwareMap);

        buildPaths();
        follower.setPose(startingPose);

        pathState = PathState.SPIN_UP_SHOOTER;
    }

    @Override
    public void loop() {
        shooter.update();
        follower.update();
        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Shooter RPM", shooter.getCurrentRPM());
        telemetry.addData("Target RPM", shooter.getTargetRPM());
        //telemetry.addData("Hood Pos", hood.getServoPos());
        telemetry.update();
    }
}
