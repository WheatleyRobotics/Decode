package org.firstinspires.ftc.teamcode.Tele;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Commands.GyroAutoAim;
import org.firstinspires.ftc.teamcode.Subsystem.Hood;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Commands.LimelightFieldDriveAssist;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class RedTele extends OpMode {
    private Follower follower;

    public static Pose startingPose = new Pose(118.37393767705385, 130.21580300719114, Math.toRadians(37));
    private boolean automatedDrive;
    private Limelight3A camera;
    private LLResult llResult;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    public Pose lastCurrentLimeLightPos = new Pose();

    private int gyroPos = 0; // RED: 0, BLUE: 180, PRACTICE: 90
    private double gyroShootPos = 100;

    private boolean lastRightTrigger = false;
    private boolean autoAimActive = false;

    private double RPMSpeed;

    private Shooter shooter;
    private Intake intake;
    private Hood hood;
    private GyroAutoAim autoAim;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(TeleConstant.startingPoseAfterAuto == null ? startingPose : TeleConstant.startingPoseAfterAuto);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(82.60623229461757, 81.17847025495752))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(37), 0.8))
                .build();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);
        autoAim = new GyroAutoAim(hardwareMap);

        // Initialize drawing offsets
        Drawing.init();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        hood.setHoodPos(TeleConstant.startingHoodPos);
        shooter.auto = false;
        camera.start();
    }

    private Pose getRobotPoseFromCamera() {
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = camera.getLatestResult().getBotpose();

            double xMeters = botpose.getPosition().x;
            double yMeters = botpose.getPosition().y;
            double yawDegrees = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

            double xInches = (xMeters + 39.3701) + 72;
            double yInches = (yMeters + 39.3701) + 72;
            double headingRadians = Math.toRadians(yawDegrees);

            return new Pose(
                    xInches,
                    yInches,
                    headingRadians);
        } else {
            return null;
        }
    }

    @Override
    public void loop() {
        // Update follower and telemetry
        follower.update();
        telemetryM.update();
        llResult = camera.getLatestResult();

        if(getRobotPoseFromCamera() != null){
            lastCurrentLimeLightPos = getRobotPoseFromCamera();
        }

        // Reset gyro if needed
        if (gamepad1.b) {
            follower.setPose(new Pose(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    Math.toRadians(gyroPos)
            ));
        }

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

        if (automatedDrive && (Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1 || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Shooter and hood controls
        if (gamepad1.right_trigger > 0.8) {
            shooter.setTargetRPM(RPMSpeed);
        } else {
            shooter.stopShooter();
        }

        if (gamepad1.dpad_up) {
            RPMSpeed = TeleConstant.bumperUpRPM;
            hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.bumperUpOffset);
            gyroShootPos = TeleConstant.bumperUpGyro;
            shooter.setIdleRPM(TeleConstant.bumperUpIdleRPM);
        } else if(gamepad1.dpad_right){
            RPMSpeed = TeleConstant.closeShotRPM;
            hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.closeShotOffset);
            gyroShootPos = TeleConstant.closeShotGyro;
            shooter.setIdleRPM(TeleConstant.closeShotIdleRPM);
        } else if (gamepad1.dpad_down) {
            RPMSpeed = TeleConstant.midShotRPM;
            hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.midShotOffset);
            gyroShootPos = TeleConstant.midShotGyro;
            shooter.setIdleRPM(TeleConstant.midIdleRPM);
        } else if (gamepad1.dpad_left) {
            RPMSpeed = TeleConstant.farShotRPM;
            hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.farShotOffset);
            gyroShootPos = TeleConstant.farShotGyro;
            shooter.setIdleRPM(TeleConstant.farIdleRPM);
        }

        boolean shooterFeeding = shooter.getTargetRPM() > 0 && shooter.isAtTargetRPM();

        if (shooterFeeding) {
            intake.intakeIn();
        } else if (gamepad1.left_bumper) {
            intake.intakeIn();
        } else if (gamepad1.y) {
            intake.intakeOut();
        } else {
            intake.stopIntaking();
            intake.stopTunnel();
        }

        shooter.update();

        if (gamepad2.y) {
            hood.manualUp();
        } else if (gamepad2.a) {
            hood.manualDown();
        }

        // =========================================================
        // =================== DRAWING CLASS USAGE =================
        // =========================================================
        Drawing.drawDebug(follower, lastCurrentLimeLightPos);

        // =========================================================
        // =================== TELEMETRY ==========================
        // =========================================================
        telemetry.addData("Shooter Ready", shooter.isAtTargetRPM());
        telemetry.addData("Target RPM", shooter.getTargetRPM());
        telemetry.addData("Current RPM", shooter.getCurrentRPM());
        telemetry.addData("RPM Difference", shooter.RPMDiff());
        telemetry.addData("Servo Pos", hood.getServoPos());
        telemetry.addData("Pinpoint Yaw (deg)", follower.getHeading());
        telemetry.addData("Target Yaw (deg)", gyroShootPos);
        telemetry.addData("Auto Aim Active", autoAimActive);
        telemetry.addData("Odometry Pose", follower.getPose());
        telemetryM.addData("Target RPM", RPMSpeed);
        telemetryM.addData("Current pose", follower.getPose());
        telemetryM.addData("Current RPM", shooter.getCurrentRPM());
        if(getRobotPoseFromCamera() != null){
            telemetry.addData("Limelight Pose:", getRobotPoseFromCamera());
            telemetryM.addData("Limelight Pose:", getRobotPoseFromCamera());
        } else {
            telemetry.addData("LimelightPose", lastCurrentLimeLightPos);
            telemetryM.addData("Limelight Pose:", lastCurrentLimeLightPos);
        }

        telemetry.update();
        telemetryM.update();
    }

    // =========================================================
    // =================== DRAWING CLASS =======================
    // =========================================================

    public static class Drawing {
        public static final double ROBOT_RADIUS = 9;
        @IgnoreConfigurable
        static PoseHistory poseHistory;
        private static final FieldManager field = PanelsField.INSTANCE.getField();

        private static final Style driveStyle = new Style("", "#3F51B5", 0.8); // Blue for odometry
        private static final Style limeStyle = new Style("", "#4CAF50", 0.8); // Green for Limelight
        private static final Style historyStyle = new Style("", "#9E9E9E", 0.8);

        public static void init() {
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        }

        public static void drawDebug(Follower follower, Pose limelightPose) {

            // Draw current path if it exists
            if (follower.getCurrentPath() != null) {
                drawPath(follower.getCurrentPath(), driveStyle);

                Pose closest = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());

                drawRobot(new Pose(
                        closest.getX(),
                        closest.getY(),
                        follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())
                ), driveStyle);
            }

            // Draw pose history safely
            PoseHistory history = follower.getPoseHistory();
            if (history != null) {
                drawPoseHistory(history);
            }

            // Draw odometry pose (blue)
            drawRobot(follower.getPose(), driveStyle);

            // Draw Limelight pose (green) as separate point
            if (limelightPose != null) {
                drawRobot(limelightPose, limeStyle);
            }

            field.update();
        }

        public static void drawRobot(Pose pose, Style style) {
            if (pose == null) return;

            field.setStyle(style);
            field.moveCursor(pose.getX(), pose.getY());
            field.circle(ROBOT_RADIUS);

            Vector heading = pose.getHeadingAsUnitVector();
            heading.setMagnitude(ROBOT_RADIUS);

            field.moveCursor(pose.getX(), pose.getY());
            field.line(
                    pose.getX() + heading.getXComponent(),
                    pose.getY() + heading.getYComponent());
        }

        public static void drawPoseHistory(PoseHistory history) {
            field.setStyle(historyStyle);

            double[] xs = history.getXPositionsArray();
            double[] ys = history.getYPositionsArray();

            for (int i = 0; i < xs.length - 1; i++) {
                field.moveCursor(xs[i], ys[i]);
                field.line(xs[i + 1], ys[i + 1]);
            }
        }

        public static void drawPath(Path path, Style style) {
            double[][] pts = path.getPanelsDrawingPoints();
            field.setStyle(style);

            for (int i = 0; i < pts[0].length - 1; i++) {
                field.moveCursor(pts[0][i], pts[1][i]);
                field.line(pts[0][i + 1], pts[1][i + 1]);
            }
        }

        public static void drawPath(PathChain chain, Style style) {
            for (int i = 0; i < chain.size(); i++) {
                drawPath(chain.getPath(i), style);
            }
        }
    }
}