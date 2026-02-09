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
import org.firstinspires.ftc.teamcode.Subsystem.LimeLight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.PoseKalmanFilter;

@Autonomous(name = "Blue Auto Vision")
public class BlueAutoVision extends OpMode {
    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private LimeLight limelight;
    private Hood hood;

    private PoseKalmanFilter poseKalman;

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
    private final int intakeBallsGyroPos = 181;

    private final Pose startingPose = new Pose(27.2, 132.283, Math.toRadians(143));
    private final Pose bumperUpPose = new Pose(27.2, 140.283, Math.toRadians(137));
    private final Pose firstIntakePose = new Pose(45.8215, 84.7384, Math.toRadians(intakeBallsGyroPos));
    private final Pose intakeFirstBallsPose = new Pose(30, 84.7323, Math.toRadians(intakeBallsGyroPos));
    private final Pose secondIntakePose = new Pose(45.8668, 61.4475, Math.toRadians(intakeBallsGyroPos));
    private final Pose intakeSecondBallsPose = new Pose(30, 59.5580, Math.toRadians(intakeBallsGyroPos));
    private final Pose thirdIntakePose = new Pose(45.8668, 35.6317, Math.toRadians(intakeBallsGyroPos));
    private final Pose intakeThirdBallsPose = new Pose(30, 35.6317, Math.toRadians(intakeBallsGyroPos));
    private final Pose endingPoint = new Pose(20, 56.8583, Math.toRadians(intakeBallsGyroPos));

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

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = new LimeLight(hardwareMap);
        hood = new Hood(hardwareMap);

        follower.setPose(startingPose);
        poseKalman = new PoseKalmanFilter(startingPose);

        buildPaths();
        pathState = PathState.SPIN_UP_SHOOTER;
    }

    @Override
    public void loop() {
        shooter.update();
        follower.update();
        limelight.update();

        Pose odomPose = follower.getPose();
        Pose visionPose = null;
        boolean hasVision = limelight.hasValidTarget();

        if (hasVision) {
            visionPose = limelight.getPedroPose();
        }

        Pose fusedPose = poseKalman.update(odomPose, visionPose, hasVision);
        follower.setPose(fusedPose);

        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Vision", hasVision);
        telemetry.addData("Pose", fusedPose);
        telemetry.update();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case SPIN_UP_SHOOTER:
                hood.setHoodPos(0.6);
                shooter.setTargetRPM(130);
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    pathState = PathState.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                shooter.setIndexerPower(1);
                intake.intakeIn();
                if (pathTimer.getElapsedTimeSeconds() > 2.7) {
                    shooter.stopShooter();
                    intake.stopIntaking();
                    pathState = PathState.GET_READY_TO_INTAKE_FIRST_BALLS;
                }
                break;

            case GET_READY_TO_INTAKE_FIRST_BALLS:
                follower.followPath(setUpIntakeFirstBallsPos, true);
                pathState = PathState.INTAKE_FIRST_BALLS;
                break;

            case INTAKE_FIRST_BALLS:
                intake.intakeIn();
                if (!follower.isBusy()) {
                    follower.followPath(intakeFirstBallsPos, true);
                    pathState = PathState.DRIVE_TO_SHOOT_FIRST_BALLS;
                }
                break;

            case DRIVE_TO_SHOOT_FIRST_BALLS:
                if (!follower.isBusy()) {
                    intake.stopIntaking();
                    follower.followPath(shootFirstBallsPos, true);
                    pathState = PathState.SHOOT_FIRST_BALLS;
                }
                break;

            case SHOOT_FIRST_BALLS:
                if (!follower.isBusy()) {
                    shooter.setTargetRPM(130);
                    shooter.setIndexerPower(1);
                    intake.intakeIn();
                    if (pathTimer.getElapsedTimeSeconds() > 6) {
                        shooter.stopShooter();
                        intake.stopIntaking();
                        pathState = PathState.GET_READY_TO_INTAKE_SECOND_BALLS;
                    }
                }
                break;

            // ===== SECOND BALLS =====
            /*
            case GET_READY_TO_INTAKE_SECOND_BALLS:
                follower.followPath(setUpIntakeSecondBallsPos, true);
                setPathState(BlueAuto.PathState.INTAKE_SECOND_BALLS);
                break;

            case INTAKE_SECOND_BALLS:
                intake.intakeIn();
                if (!follower.isBusy()) {
                    follower.followPath(intakeSecondBallsPos, true);
                    setPathState(BlueAuto.PathState.DRIVE_TO_SHOOT_SECOND_BALLS);
                }
                break;

            case DRIVE_TO_SHOOT_SECOND_BALLS:
                if (!follower.isBusy()) {
                    intake.stopIntaking();
                    follower.followPath(shootSecondBallsPos, true);
                    setPathState(BlueAuto.PathState.SHOOT_SECOND_BALLS);
                }
                break;

            case SHOOT_SECOND_BALLS:
                if (!follower.isBusy() && shootDelayTimer.getElapsedTimeSeconds() > 0.3) {
                    shooter.setTargetRPM(130);
                    intake.intakeIn();
                    shooter.setIndexerPower(1);
                    if (pathTimer.getElapsedTimeSeconds() > timeToShoot) {
                        shooter.stopShooter();
                        intake.stopIntaking();
                        setPathState(BlueAuto.PathState.GET_READY_TO_INTAKE_THIRD_BALLS);
                    }
                }
                break;

            // ===== THIRD BALLS =====
            case GET_READY_TO_INTAKE_THIRD_BALLS:
                follower.followPath(setUpIntakeThirdBallsPos, true);
                setPathState(BlueAuto.PathState.INTAKE_THIRD_BALLS);
                break;

            case INTAKE_THIRD_BALLS:
                intake.intakeIn();
                if (!follower.isBusy()) {
                    follower.followPath(intakeThirdBallsPos, true);
                    setPathState(BlueAuto.PathState.DRIVE_TO_SHOOT_THIRD_BALLS);
                }
                break;

            case DRIVE_TO_SHOOT_THIRD_BALLS:
                if (!follower.isBusy()) {
                    intake.stopIntaking();
                    follower.followPath(shootThirdBallsPos, true);
                    setPathState(BlueAuto.PathState.SHOOT_THIRD_BALLS);
                }
                break;

            case SHOOT_THIRD_BALLS:
                if (!follower.isBusy() && shootDelayTimer.getElapsedTimeSeconds() > 0.3) {
                    shooter.setTargetRPM(130);
                    intake.intakeIn();
                    shooter.setIndexerPower(1);
                    if (pathTimer.getElapsedTimeSeconds() > timeToShoot) {
                        shooter.stopShooter();
                        intake.stopIntaking();
                        setPathState(BlueAuto.PathState.ENDPOS);
                    }
                }
                break;

            case ENDPOS:
                //hood.setHoodPos(0);
                follower.followPath(end, true);
                setPathState(BlueAuto.PathState.DONE);
                break;

             */

            case DONE:
                break;
        }
    }
}
