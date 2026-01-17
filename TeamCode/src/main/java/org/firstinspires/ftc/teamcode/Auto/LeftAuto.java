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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Left Auto")
public class LeftAuto extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Hood hood;

    private Timer pathTimer;

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

    private final int intakeBallsGyroPos = 181;

    // ===== POSES (UNCHANGED) =====
    private final Pose startingPose =
            new Pose(20.996923076923085, 119.64923076923078, Math.toRadians(140));

    private final Pose firstIntakePose =
            new Pose(58.43692307692309, 84.73846153846152, Math.toRadians(intakeBallsGyroPos));
    private final Pose intakeFirstBallsPose =
            new Pose(25, 84.8243626062323, Math.toRadians(intakeBallsGyroPos));

    private final Pose secondIntakePose =
            new Pose(43.86685552407931, 59.44759206798866, Math.toRadians(intakeBallsGyroPos));
    private final Pose intakeSecondBallsPose =
            new Pose(25, 59.55807365439092, Math.toRadians(intakeBallsGyroPos));

    private final Pose thirdIntakePose =
            new Pose(43.86685552407931, 35.63172804532578, Math.toRadians(intakeBallsGyroPos));
    private final Pose intakeThirdBallsPose =
            new Pose(25, 35.63172804532578, Math.toRadians(intakeBallsGyroPos));

    private final Pose endingPoint =
            new Pose(17.906515580736542, 56.8583569405099, Math.toRadians(intakeBallsGyroPos));

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
                .setLinearHeadingInterpolation(intakeFirstBallsPose.getHeading(), startingPose.getHeading())
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
                .setLinearHeadingInterpolation(intakeSecondBallsPose.getHeading(), startingPose.getHeading())
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
                .setLinearHeadingInterpolation(intakeThirdBallsPose.getHeading(), startingPose.getHeading())
                .build();

        end = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, endingPoint))
                .setLinearHeadingInterpolation(startingPose.getHeading(), endingPoint.getHeading())
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {

        switch (pathState) {

            case SPIN_UP_SHOOTER:
                hood.setHoodPos(0.6);
                shooter.setTargetRPM(130);
                //if (shooter.isAtTargetRPM() || pathTimer.getElapsedTimeSeconds() > 0.8) {
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                shooter.setIndexerPower(1);
                intake.intakeIn();
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    shooter.stopShooter();
                    intake.stopIntaking();
                    setPathState(PathState.GET_READY_TO_INTAKE_FIRST_BALLS);
                }
                break;

            // ===== FIRST BALLS =====
            case GET_READY_TO_INTAKE_FIRST_BALLS:
                if (!follower.isBusy()) {
                    follower.followPath(setUpIntakeFirstBallsPos, true);
                    setPathState(PathState.INTAKE_FIRST_BALLS);
                }
                break;

            case INTAKE_FIRST_BALLS:
                if (!follower.isBusy()) {
                    intake.intakeIn();
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
                shooter.setTargetRPM(130);
                if (shooter.isAtTargetRPM() || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    shooter.setIndexerPower(1);
                    intake.intakeIn();
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
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
                if (!follower.isBusy()) {
                    intake.intakeIn();
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
                shooter.setTargetRPM(130);
                if (shooter.isAtTargetRPM() || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    shooter.setIndexerPower(1);
                    intake.intakeIn();
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
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
                if (!follower.isBusy()) {
                    intake.intakeIn();
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
                shooter.setTargetRPM(130);
                if (shooter.isAtTargetRPM() || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    shooter.setIndexerPower(1);
                    intake.intakeIn();
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        shooter.stopShooter();
                        intake.stopIntaking();
                        setPathState(PathState.ENDPOS);
                    }
                }
                break;

            case ENDPOS:
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
        hood = new Hood(hardwareMap);

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
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }
}