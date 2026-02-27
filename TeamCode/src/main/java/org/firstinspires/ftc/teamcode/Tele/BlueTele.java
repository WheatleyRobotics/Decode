package org.firstinspires.ftc.teamcode.Tele;

import static org.firstinspires.ftc.teamcode.Tele.TeleConstant.GyroOffsets;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.GyroAutoAim;
import org.firstinspires.ftc.teamcode.Subsystem.Hood;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
//import org.firstinspires.ftc.teamcode.Commands.LimelightFieldDriveAssist;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.LimeLight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class BlueTele extends OpMode {
    private Follower follower;

    public static Pose startingPose = new Pose(25.855524079320112, 130.38810198300283, Math.toRadians(143));
    private boolean automatedDrive;

    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private Drawing drawing;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Limelight
    private LimeLight limelight;
    public Pose lastCurrentLimeLightPos = new Pose();

    private int gyroPos = 196; // RED: 20, BLUE: 195, PRACTICE: 110
    private double gyroShootPos = 100;

    private boolean lastRightTrigger = false;
    private boolean autoAimActive = false;
    boolean visionMode = true;
    boolean lastToggle = false;
    ElapsedTime intakeOutTimer = new ElapsedTime();
    boolean intakeOutActive = false;
    boolean lastTrigger;

    private double RPMSpeed;

    private Shooter shooter;
    private Intake intake;
    private Hood hood;
    private GyroAutoAim autoAim;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Limelight init
        autoAim = new GyroAutoAim(hardwareMap);
        limelight = new LimeLight(hardwareMap, autoAim);

        follower.setStartingPose(TeleConstant.startingPoseAfterAuto == null
                ? startingPose
                : TeleConstant.startingPoseAfterAuto);

        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() // Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(54, 92))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading,
                        Math.toRadians(134), //gyroPos + GyroOffsets)
                        0.8))
                .build();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);
        autoAim = new GyroAutoAim(hardwareMap);

        // Initialize drawing offsets
        drawing.init();

        hood.setRange(0.29, 0.67);

        limelight.setAllianceRed(false);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        hood.setHoodPos(TeleConstant.startingHoodPos);
        shooter.auto = false;

        // Start Limelight
        limelight.start();
    }

    @Override
    public void loop() {
        // Update follower and telemetry
        follower.update();
        telemetryM.update();

        // Update Limelight pose
        Pose camPose = limelight.getPose();
        if (camPose != null) {
            lastCurrentLimeLightPos = camPose;
        }

        if(gamepad1.b){
            autoAim.resetGyro(gyroPos);
        }

        /*
        // Reset gyro if needed
        if (gamepad1.b) {
            follower.setPose(new Pose(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    Math.toRadians(gyroPos)
            ));
        }

         */

        // TeleOp drive controls
        if (!automatedDrive) {
            if (!slowMode) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false,
                        Math.toRadians(gyroPos)
                );
            } else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        false,
                        Math.toRadians(gyroPos)
                );
            }
        }

        // Automated PathFollowing
        if (gamepad1.left_trigger > 0.8) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (
                Math.abs(gamepad1.left_stick_y) > 0.1 ||
                        Math.abs(gamepad1.left_stick_x) > 0.1 ||
                        Math.abs(gamepad1.right_stick_x) > 0.1 ||
                        !follower.isBusy())) {

            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Shooter and hood controls
        // Declare this at the top of your OpMode class

        if (gamepad2.a && !lastToggle) {
            visionMode = !visionMode;
        }

        lastToggle = gamepad2.a;

        // Switch based on visionMode
        switch (visionMode ? 1 : 2) {
            case 1: // Vision Mode
                if (gamepad1.right_trigger > 0.8) {
                    if (limelight.hasTarget()) {
                        RPMSpeed = shooter.RPMSpeed(limelight.distanceFromTagInches());
                        shooter.setTargetRPM(RPMSpeed);
                    } else {
                        shooter.setTargetRPM(TeleConstant.bumperUpRPM); // keep last speed if no target
                    }
                } else {
                    shooter.stopShooter();
                }

                if (limelight.hasTarget()) {
                    double hoodPos = hood.hoodPosVision(limelight.distanceFromTagInches());
                    hood.setHoodPos(hoodPos);
                }
                else{
                    hood.setHoodPos(TeleConstant.startingHoodPos+TeleConstant.bumperUpOffset);
                }
                break;

            case 2: // Non-Vision Mode
                if (gamepad1.right_trigger > 0.8) {
                    shooter.setTargetRPM(RPMSpeed);
                } else {
                    shooter.stopShooter();
                }

                if (gamepad2.dpad_up) {
                    RPMSpeed = TeleConstant.bumperUpRPM;
                    hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.bumperUpOffset);
                    gyroShootPos = TeleConstant.bumperUpGyro;
                    shooter.setIdleRPM(TeleConstant.bumperUpIdleRPM);
                } else if (gamepad2.dpad_right) {
                    RPMSpeed = TeleConstant.closeShotRPM;
                    hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.closeShotOffset);
                    gyroShootPos = TeleConstant.closeShotGyro;
                    shooter.setIdleRPM(TeleConstant.closeShotIdleRPM);
                } else if (gamepad2.dpad_down) {
                    RPMSpeed = TeleConstant.midShotRPM;
                    hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.midShotOffset);
                    gyroShootPos = TeleConstant.midShotGyro;
                    shooter.setIdleRPM(TeleConstant.midIdleRPM);
                } else if (gamepad2.dpad_left) {
                    RPMSpeed = TeleConstant.farShotRPM;
                    hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.farShotOffset);
                    gyroShootPos = TeleConstant.farShotGyro;
                    shooter.setIdleRPM(TeleConstant.farIdleRPM);
                }
                break;
        }

        boolean triggerPressed = gamepad1.right_trigger > 0.8;

        if (triggerPressed && !lastTrigger && !intakeOutActive) {
            intakeOutActive = true;
            intakeOutTimer.reset();
        }

        lastTrigger = triggerPressed;

        boolean shooterFeeding = shooter.getTargetRPM() > 0 && shooter.isAtTargetRPM();
        boolean prepShooter = shooter.getTargetRPM() > 0;

        // BURST ACTIVE
        if (intakeOutActive) {
            intake.intakeOut();

            if (intakeOutTimer.seconds() >= 0.1) {
                intakeOutActive = false;
            }
        }
        else {
            if (shooterFeeding) {
                intake.intakeIn();
                shooter.setIndexerPower(1);
            }
            else if (prepShooter) {
                intake.prepShooter();
                shooter.setIndexerPower(-1);
            }
            else if (gamepad1.left_bumper) {
                intake.intakeIn();
            }
            else if (gamepad1.y) {
                intake.intakeOut();
            }
            else {
                intake.stopIntaking();
                intake.stopTunnel();
            }
        }

        /*
        if (shooterFeeding) {
            intake.intakeIn();
            shooter.setIndexerPower(1);
        }
        else if(prepShooter){
            intake.prepShooter();
            shooter.setIndexerPower(-1);
        }
        else if (gamepad1.left_bumper) {
            intake.intakeIn();
        } else if (gamepad1.y) {
            intake.intakeOut();
        } else {
            intake.stopIntaking();
            intake.stopTunnel();
        }

         */

        shooter.update();

        /*
        if (gamepad2.y) {
            hood.manualUp();
        } else if (gamepad2.a) {
            hood.manualDown();
        }
         */

        if(gamepad2.yWasPressed()){
            GyroOffsets += 10;
        }
        else if(gamepad2.aWasPressed()){
            GyroOffsets -= 10;
        }

        // =========================================================
        // =================== DRAWING CLASS USAGE =================
        // =========================================================
        drawing.drawDebug(follower, lastCurrentLimeLightPos);

        // =========================================================
        // =================== TELEMETRY ==========================
        // =========================================================
        telemetry.addData("Shooter Ready", shooter.isAtTargetRPM());
        telemetry.addData("Target RPM", shooter.getTargetRPM());
        telemetry.addData("Current RPM", shooter.getCurrentRPM());
        telemetry.addData("RPM Difference", shooter.RPMDiff());
        telemetry.addData("Servo Pos", hood.getServoPos());
        telemetry.addData("Pinpoint Yaw (deg)", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Target Yaw (deg)", gyroShootPos);
        telemetry.addData("Auto Aim Active", autoAimActive);
        telemetry.addData("Odometry Pose", follower.getPose());
        telemetry.addData("target auto score: ", -(gyroPos + GyroOffsets));

        telemetryM.addData("Target RPM", RPMSpeed);
        telemetryM.addData("Current pose", follower.getPose());
        telemetryM.addData("Current RPM", shooter.getCurrentRPM());

        if (limelight.hasTarget()) {
            telemetry.addData("Limelight Pose", limelight.getLastPose());
            telemetryM.addData("Limelight Pose", limelight.getLastPose());
            telemetry.addData("Distance", limelight.distanceFromTagInches());
        } else {
            telemetry.addData("Last LL Pose", lastCurrentLimeLightPos);
            telemetryM.addData("Last LL Pose", lastCurrentLimeLightPos);
        }

        telemetry.update();
        telemetryM.update();
    }
}