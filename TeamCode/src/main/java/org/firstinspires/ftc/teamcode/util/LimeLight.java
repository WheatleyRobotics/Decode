package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Commands.GyroAutoAim;

import com.pedropathing.geometry.Pose;

public class LimeLight {

    private Limelight3A camera;
    private LLResult result;

    private GyroAutoAim autoAim;

    private Pose lastPose = new Pose();
    private double ty;

    // alliance control
    private boolean isRedAlliance = false;

    public LimeLight(HardwareMap hardwareMap, GyroAutoAim autoAim) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        this.autoAim = autoAim;
    }

    /** Start camera stream */
    public void start() {
        camera.start();
    }

    /** Stop camera stream */
    public void stop() {
        camera.stop();
    }

    /** Set alliance for coordinate mirroring */
    public void setAllianceRed(boolean red) {
        isRedAlliance = red;
    }

    /** Returns latest pose if valid, otherwise null */
    public Pose getPose() {
        result = camera.getLatestResult();

        if (result != null && result.isValid()) {

            Pose3D botpose = result.getBotpose();

            // meters → inches, shift origin to field center
            double xInches = -botpose.getPosition().x * 39.3701 + 72;
            double yInches = -botpose.getPosition().y * 39.3701 + 72;

            // convert yaw to heading radians
            /*
            double headingRadians =
                    Math.toRadians(-botpose.getOrientation().getYaw(AngleUnit.DEGREES));

             */

            // mirror pose if RED alliance
            if (isRedAlliance) {
                xInches = 144 - xInches;
                yInches = 144 - yInches;
                //headingRadians += Math.PI;
            }

            lastPose = new Pose(
                    xInches,
                    yInches,
                    autoAim.getYaw()
                    //headingRadians
            );

            return lastPose;
        }

        return null;
    }

    /** Returns last valid pose (never null) */
    public Pose getLastPose() {
        return lastPose;
    }

    public double getTagDistanceInches() {
        result = camera.getLatestResult();

        if (result != null && result.isValid()) {
            return (result.getBotposeAvgDist() * 39.3701) - 11.82;
        }

        return 0;
    }

    /** Manually set the pose (for gyro reset or calibration) */
    public void setPose(Pose pose) {
        if (pose != null) {
            lastPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
        }
    }

    /** Returns true if camera currently sees tags */
    public boolean hasTarget() {
        result = camera.getLatestResult();
        return result != null && result.isValid();
    }
}