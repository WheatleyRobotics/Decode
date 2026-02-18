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

@Autonomous(name = "Bumper Up Blue Auto")
public class BumperUpBlueAuto extends OpMode {
    private final AutoConstants autoConstants = new AutoConstants();
    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Hood hood;

    private Timer pathTimer;
    private Timer indexerTimer;
    private Timer shootDelayTimer;

    public enum PathState {
        SPIN_UP_SHOOTER,
        SHOOT_PRELOAD,

        GET_READY_TO_INTAKE_FIRST_BALLS,
        INTAKE_FIRST_BALLS,
        AVOID_BANG,
        DRIVE_TO_SHOOT_FIRST_BALLS,
        SHOOT_FIRST_BALLS,

        GET_READY_TO_INTAKE_SECOND_BALLS,
        INTAKE_SECOND_BALLS,
        AVOID_WALL,
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
    private final Pose startingPose = new Pose(23, 126.92067988668555, Math.toRadians(141)); // x: 25.976203966005663, y: 130.24333994334273 gyro 143
    private final Pose bumperUpPose = new Pose(22, 126.92067988668555, Math.toRadians(141)); //x: 23 y: 126.92067988668555
    private final Pose firstIntakePose = new Pose(59, 90, Math.toRadians(intakeBallsGyroPos)); //x: 57 y: 88
    private final Pose intakeFirstBallsPose = new Pose(29, 88, Math.toRadians(intakeBallsGyroPos));
    private final Pose avoidBangPose = new Pose(24.14050991501417, 99.64843909348437, Math.toRadians(141));
    private final Pose secondIntakePose = new Pose(58, 65, Math.toRadians(intakeBallsGyroPos)); //y:61
    private final Pose avoidWallPose = new Pose(45, 61, Math.toRadians(intakeBallsGyroPos));
    private final Pose intakeSecondBallsPose = new Pose(25, 59.5580, Math.toRadians(intakeBallsGyroPos));
    private final Pose thirdIntakePose = new Pose(57, 35.6317, Math.toRadians(intakeBallsGyroPos));
    private final Pose intakeThirdBallsPose = new Pose(30, 35.6317, Math.toRadians(intakeBallsGyroPos));
    private final Pose endingPoint = new Pose(20, 56.8583, Math.toRadians(intakeBallsGyroPos));

    private PathChain setUpIntakeFirstBallsPos, intakeFirstBallsPos, avoidBangPos, shootFirstBallsPos,
            setUpIntakeSecondBallsPos, intakeSecondBallsPos, avoidWallPos, shootSecondBallsPos,
            setUpIntakeThirdBallsPos, intakeThirdBallsPos, shootThirdBallsPos,
            end;

    public void buildPaths() {
        setUpIntakeFirstBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, firstIntakePose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), firstIntakePose.getHeading())
                .build();

        intakeFirstBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(firstIntakePose, intakeFirstBallsPose)) // robot moves to intake
                .setLinearHeadingInterpolation(firstIntakePose.getHeading(), intakeFirstBallsPose.getHeading())
                .build();

        avoidBangPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeFirstBallsPose, avoidBangPose)) // robot moves after intake
                .setLinearHeadingInterpolation(intakeFirstBallsPose.getHeading(), avoidBangPose.getHeading())
                .build();

        shootFirstBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(avoidBangPose, bumperUpPose))
                .setLinearHeadingInterpolation(avoidBangPose.getHeading(), bumperUpPose.getHeading())
                .build();

        setUpIntakeSecondBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(bumperUpPose, secondIntakePose))
                .setLinearHeadingInterpolation(bumperUpPose.getHeading(), secondIntakePose.getHeading())
                .build();

        intakeSecondBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(secondIntakePose, intakeSecondBallsPose))
                .setLinearHeadingInterpolation(secondIntakePose.getHeading(), intakeSecondBallsPose.getHeading())
                .build();

        avoidWallPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeSecondBallsPose, avoidWallPose))
                .setLinearHeadingInterpolation(intakeSecondBallsPose.getHeading(), avoidWallPose.getHeading())
                .build();

        shootSecondBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(avoidWallPose, bumperUpPose))
                .setLinearHeadingInterpolation(avoidWallPose.getHeading(), bumperUpPose.getHeading())
                .build();

        setUpIntakeThirdBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(bumperUpPose, thirdIntakePose))
                .setLinearHeadingInterpolation(bumperUpPose.getHeading(), thirdIntakePose.getHeading())
                .build();

        intakeThirdBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(thirdIntakePose, intakeThirdBallsPose))
                .setLinearHeadingInterpolation(thirdIntakePose.getHeading(), intakeThirdBallsPose.getHeading())
                .build();

        shootThirdBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeThirdBallsPose, bumperUpPose))
                .setLinearHeadingInterpolation(intakeThirdBallsPose.getHeading(), bumperUpPose.getHeading())
                .build();

        end = follower.pathBuilder()
                .addPath(new BezierLine(bumperUpPose, endingPoint))
                .setLinearHeadingInterpolation(bumperUpPose.getHeading(), endingPoint.getHeading())
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        if (newState.toString().startsWith("SHOOT_")) {
            shootDelayTimer = new Timer();
            indexerTimer = new Timer();
        }
    }

    public void statePathUpdate() {
        switch (pathState) {
            case SPIN_UP_SHOOTER:
                hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.bumperUpOffset);
                shooter.setTargetRPM(TeleConstant.bumperUpRPM);
                if (pathTimer.getElapsedTimeSeconds() > autoConstants.preLoadOnlySpinUpShooterWait) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if(shooter.isAtTargetRPM()) {
                    shooter.setIndexerPower(autoConstants.indexerPower); // feed balls
                    intake.intakeIn();
                }
                if (pathTimer.getElapsedTimeSeconds() > autoConstants.shootPreLoadBalls) {
                    shooter.setIndexerPower(-autoConstants.indexerPower);
                    shooter.autoShooter();
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
                    setPathState(PathState.AVOID_BANG);
                }
                break;


            case AVOID_BANG:
                if (!follower.isBusy()) {
                    follower.followPath(avoidBangPos, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_FIRST_BALLS);
                }
                break;

            case DRIVE_TO_SHOOT_FIRST_BALLS:
                shooter.setIndexerPower(-autoConstants.indexerPower);

                if (!follower.isBusy()) {
                    intake.stopTunnel();
                    intake.onlyIntake();
                    hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.bumperUpOffset);
                    follower.followPath(shootFirstBallsPos, true);
                    setPathState(PathState.SHOOT_FIRST_BALLS);
                }
                break;

            case SHOOT_FIRST_BALLS:
                shooter.setTargetRPM(TeleConstant.bumperUpRPM);

                if (!follower.isBusy()) {
                    double shootElapsed = shootDelayTimer.getElapsedTimeSeconds();

                    if (shootElapsed < autoConstants.indexerShootingDelay) {
                        shooter.setIndexerPower(-1); // wait before feeding
                        intake.onlyIntake();
                    } else if (shootElapsed < autoConstants.shootingTimer) {
                        if(shooter.isAtTargetRPM()) {
                            shooter.setIndexerPower(autoConstants.indexerPower); // feed balls
                            intake.intakeIn();
                        }
                    } else {
                        shooter.setIndexerPower(-1); // stop after shooting window
                        intake.stopIntaking();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > autoConstants.timeToShoot) {
                        shooter.autoShooter();
                        intake.stopIntaking();
                        setPathState(PathState.GET_READY_TO_INTAKE_SECOND_BALLS);
                    }
                }
                break;

            // ===== SECOND BALLS =====
            case GET_READY_TO_INTAKE_SECOND_BALLS:
                //shooter.setIndexerPower(-autoConstants.indexerPower);
                if (!follower.isBusy()) {
                    follower.followPath(setUpIntakeSecondBallsPos, true);
                    setPathState(PathState.INTAKE_SECOND_BALLS);
                }
                break;

            case INTAKE_SECOND_BALLS:
                intake.intakeIn();
                if (!follower.isBusy()) {
                    follower.followPath(intakeSecondBallsPos, true);
                    intake.stopTunnel();
                    setPathState(PathState.AVOID_WALL);
                }
                break;

            case AVOID_WALL:
                intake.onlyIntake();
                if (!follower.isBusy()) {
                    follower.followPath(avoidWallPos, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_SECOND_BALLS);
                }
                break;

            case DRIVE_TO_SHOOT_SECOND_BALLS:
                double indexerDelay = indexerTimer.getElapsedTimeSeconds();
                if (indexerDelay < autoConstants.indexerIntake) {
                    intake.intakeIn();
                }
                else {
                    intake.stopTunnel();
                }

                if (!follower.isBusy()) {
                    intake.onlyIntake();
                    hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.bumperUpOffset);
                    follower.followPath(shootSecondBallsPos, true);
                    setPathState(PathState.SHOOT_SECOND_BALLS);
                }
                break;

            case SHOOT_SECOND_BALLS:
                shooter.setTargetRPM(TeleConstant.bumperUpRPM);

                if (!follower.isBusy()) {
                    double shootElapsed = shootDelayTimer.getElapsedTimeSeconds();

                    if (shootElapsed < autoConstants.indexerShootingDelay) {
                        shooter.setIndexerPower(-1); // wait before feeding
                        intake.onlyIntake();
                    } else if (shootElapsed < autoConstants.shootingTimer) {
                        if(shooter.isAtTargetRPM()) {
                            shooter.setIndexerPower(autoConstants.indexerPower); // feed balls
                            intake.intakeIn();
                        }
                    } else {
                        shooter.setIndexerPower(-1); // stop after shooting window
                        intake.stopIntaking();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > autoConstants.timeToShoot) {
                        shooter.autoShooter();
                        intake.stopIntaking();
                        setPathState(PathState.GET_READY_TO_INTAKE_SECOND_BALLS);
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
                    intake.onlyIntake();
                    hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.bumperUpOffset);
                    follower.followPath(shootThirdBallsPos, true);
                    setPathState(PathState.SHOOT_THIRD_BALLS);
                }
                break;

            case SHOOT_THIRD_BALLS:
                shooter.setTargetRPM(TeleConstant.bumperUpRPM);

                if (!follower.isBusy()) {
                    if (shootDelayTimer.getElapsedTimeSeconds() > autoConstants.indexerShootingDelay) {
                        if (shootDelayTimer.getElapsedTimeSeconds() > autoConstants.shootingTimer) {
                            shooter.setIndexerPower(autoConstants.indexerPower);
                            intake.intakeIn();
                        }
                    }

                    if (pathTimer.getElapsedTimeSeconds() > autoConstants.timeToShoot) {
                        shooter.setIndexerPower(-autoConstants.indexerPower);
                        shooter.autoShooter();
                        intake.stopIntaking();
                        setPathState(PathState.GET_READY_TO_INTAKE_SECOND_BALLS);
                    }
                }
                break;

            case ENDPOS:
                hood.setHoodPos(TeleConstant.startingHoodPos);
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

        shooter.auto = true;
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

    @Override
    public void stop(){
        TeleConstant.startingPoseAfterAuto = follower.getPose();
    }
}