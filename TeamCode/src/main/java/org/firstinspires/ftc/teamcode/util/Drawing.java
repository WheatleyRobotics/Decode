package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

public class Drawing {
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