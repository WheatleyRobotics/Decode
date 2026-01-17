package org.firstinspires.ftc.teamcode.Auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Hood;

@Autonomous(name = "Right Auto Shoot+Leave")
public class Right_Auto_Shoot_and_Leave extends OpMode {
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
            new Pose(123.38785792111572, 121.64923076923078, Math.toRadians(35));

    private final Pose intakePose =
            new Pose(97.96033994334275, 84.73846153846152, Math.toRadians(0));

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
                hood.setHoodPos(0.6);
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
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);


        buildPaths();
        follower.setPose(startingPose);


        pathState = PathState.SPIN_UP_SHOOTER;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.SPIN_UP_SHOOTER);
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