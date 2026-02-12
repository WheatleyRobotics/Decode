package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//import org.firstinspires.ftc.teamcode.Subsystem.Hood;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Red Auto")
public class RedAuto extends OpMode {
    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    //private Hood hood;
    private AutoConstants autoConstants;

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
    private final int intakeBallsGyroPos = 0;


    // ===== POSES (UNCHANGED) =====
    private final Pose startingPose = new Pose(125.30878186968842, 120.83336674656788, Math.toRadians(35));
    private final Pose bumperUpPose = new Pose(123.56, 119.22153846153844, Math.toRadians(35));
    private final Pose firstIntakePose = new Pose(96.81019830028329, 80.7384, Math.toRadians(intakeBallsGyroPos));
    private final Pose intakeFirstBallsPose = new Pose(132, 84.7323, Math.toRadians(intakeBallsGyroPos));
    private final Pose secondIntakePose = new Pose(98.12181303116145, 59.29846153846154, Math.toRadians(intakeBallsGyroPos));
    private final Pose intakeSecondBallsPose = new Pose(128, 58.94617563739374, Math.toRadians(intakeBallsGyroPos));
    private final Pose thirdIntakePose = new Pose(102.39093484419263, 30.22379603399434, Math.toRadians(intakeBallsGyroPos));
    private final Pose intakeThirdBallsPose = new Pose(132.9830028328612, 34.61189801699716, Math.toRadians(intakeBallsGyroPos));
    private final Pose endingPoint = new Pose(131.71954674220962, 58.49008498583567, Math.toRadians(intakeBallsGyroPos));


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
                .addPath(new BezierLine(intakeFirstBallsPose, bumperUpPose))
                .setLinearHeadingInterpolation(intakeFirstBallsPose.getHeading(), bumperUpPose.getHeading())
                .build();


        setUpIntakeSecondBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(bumperUpPose, secondIntakePose))
                .setLinearHeadingInterpolation(bumperUpPose.getHeading(), secondIntakePose.getHeading())
                .build();


        intakeSecondBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(secondIntakePose, intakeSecondBallsPose))
                .setLinearHeadingInterpolation(secondIntakePose.getHeading(), intakeSecondBallsPose.getHeading())
                .build();


        shootSecondBallsPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeSecondBallsPose, bumperUpPose))
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
                .addPath(new BezierLine(intakeThirdBallsPose, bumperUpPose))
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

    private final double preLoadWait = 2;
    private final double timeToShoot = 6;

    public void statePathUpdate() {
        switch (pathState) {
            case SPIN_UP_SHOOTER:
                //hood.setHoodPos(0.6);
                shooter.setTargetRPM(130);
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;


            case SHOOT_PRELOAD:
                shooter.setIndexerPower(1);
                intake.intakeIn();
                if (pathTimer.getElapsedTimeSeconds() > 2.7) {
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
                if (!follower.isBusy() && shootDelayTimer.getElapsedTimeSeconds() > 0.3) {
                    shooter.setTargetRPM(130);
                    intake.intakeIn();
                    shooter.setIndexerPower(1);
                    if (pathTimer.getElapsedTimeSeconds() > timeToShoot) {
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
                if (!follower.isBusy() && shootDelayTimer.getElapsedTimeSeconds() > 0.3) {
                    shooter.setTargetRPM(130);
                    intake.intakeIn();
                    shooter.setIndexerPower(1);
                    if (pathTimer.getElapsedTimeSeconds() > timeToShoot) {
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
                if (!follower.isBusy() && shootDelayTimer.getElapsedTimeSeconds() > 0.3) {
                    shooter.setTargetRPM(130);
                    intake.intakeIn();
                    shooter.setIndexerPower(1);
                    if (pathTimer.getElapsedTimeSeconds() > timeToShoot) {
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
}

