package org.firstinspires.ftc.teamcode.Tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.*;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class GraphPose extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(118.37393767705385, 130.21580300719114, Math.toRadians(37));

    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize drawing
        Drawing.init();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(45),
                                0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        follower.update();

        // Draw robot + path + pose history
        Drawing.drawDebug(follower);

        telemetryM.update();

        if (!automatedDrive) {
            if (!slowMode) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true
                );
            } else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        true
                );
            }
        }

        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }

    // =========================================================
    // =================== DRAWING CLASS =======================
    // =========================================================

    public static class Drawing {
        public static final double ROBOT_RADIUS = 9;
        @IgnoreConfigurable
        static PoseHistory poseHistory;
        private static final FieldManager field = PanelsField.INSTANCE.getField();

        private static final Style robotStyle = new Style("", "#3F51B5", 0.8);
        private static final Style historyStyle = new Style("", "#4CAF50", 0.8);

        public static void init() {
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        }

        public static void drawDebug(Follower follower) {

            // Draw current path if exists
            if (follower.getCurrentPath() != null) {
                drawPath(follower.getCurrentPath(), robotStyle);

                Pose closest = follower.getPointFromPath(
                        follower.getCurrentPath().getClosestPointTValue());

                drawRobot(new Pose(
                                closest.getX(),
                                closest.getY(),
                                follower.getCurrentPath().getHeadingGoal(
                                        follower.getCurrentPath().getClosestPointTValue())),
                        robotStyle);
            }

            // Draw pose history safely
            PoseHistory history = follower.getPoseHistory();
            if (history != null) {
                drawPoseHistory(history);
            }

            // Draw current robot pose
            drawRobot(follower.getPose());

            field.update();
        }

        public static void drawRobot(Pose pose) {
            drawRobot(pose, robotStyle);
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