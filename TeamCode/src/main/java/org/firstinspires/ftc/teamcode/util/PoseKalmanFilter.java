package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class PoseKalmanFilter {

    private final Kalman1D x;
    private final Kalman1D y;
    private final Kalman1D h;

    public PoseKalmanFilter(Pose initialPose) {
        x = new Kalman1D(initialPose.getX(), 1, 0.15, 2.5);
        y = new Kalman1D(initialPose.getY(), 1, 0.15, 2.5);
        h = new Kalman1D(initialPose.getHeading(), 1, 0.05, 1.5);
    }

    public Pose update(Pose odomPose, Pose visionPose, boolean hasVision) {

        double mx = hasVision ? visionPose.getX() : odomPose.getX();
        double my = hasVision ? visionPose.getY() : odomPose.getY();
        double mh = hasVision ? visionPose.getHeading() : odomPose.getHeading();

        return new Pose(
                x.update(mx),
                y.update(my),
                h.update(mh)
        );
    }

    private static class Kalman1D {
        private double estimate;
        private double error;
        private final double q;
        private final double r;

        Kalman1D(double estimate, double error, double q, double r) {
            this.estimate = estimate;
            this.error = error;
            this.q = q;
            this.r = r;
        }

        double update(double measurement) {
            error += q;
            double k = error / (error + r);
            estimate += k * (measurement - estimate);
            error *= (1 - k);
            return estimate;
        }
    }
}
