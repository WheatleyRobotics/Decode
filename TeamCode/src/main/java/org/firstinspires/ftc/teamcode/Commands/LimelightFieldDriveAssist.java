//package org.firstinspires.ftc.teamcode.Commands;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.pedropathing.ftc.FTCCoordinates;
//import com.pedropathing.geometry.PedroCoordinates;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.Subsystem.LimeLight;
//
//public class LimelightFieldDriveAssist {
//    private LimeLight limeLight;
//    private Follower follower;
//    private GoBildaPinpointDriver pinpoint;
//
//    private Pose targetPose = new Pose();
//    private boolean following = false;
//    private double targetHeadingRad = 0;
//
//    private PathChain activePath = null;
//
//    public LimelightFieldDriveAssist(LimeLight limeLight, Follower follower, HardwareMap hardwareMap) {
//        this.limeLight = limeLight;
//        this.follower = follower;
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "OdometryComputer");
//    }
//
//    // set scoring target
//    public void setTarget(Pose pose, double headingDeg) {
//        targetPose = pose;
//        targetHeadingRad = Math.toRadians(headingDeg);
//    }
//
//    /**
//     * Starts auto alignment path
//     */
//    public void autoAlign(){
//        if (!following) {
//            follower.followPath(
//                    follower.pathBuilder()
//                            .addPath(new BezierLine(follower.getPose(), targetPose))
//                            .setLinearHeadingInterpolation(follower.getHeading(), targetPose.minus(follower.getPose()).getAsVector().getTheta())
//                            .build()
//            );
//        }
//        //This uses the aprilTag to relocalize your robot
//        //You can also create a custom AprilTag fusion Localizer for the follower if you want to use this by default for all your autos
//        follower.setPose(getRobotPoseFromCamera());
//        if (following && !follower.isBusy()) following = false;
//    }
//
//
//    private Pose getRobotPoseFromCamera() {
//        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
//        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
//        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
//        return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
//    }
//
//    /**
//     * Returns true while path is running
//     */
//    public boolean isBusy() {
//        return follower.isBusy();
//    }
//
//    // ===============================
//    // PINPOINT GYRO HELPERS (KEEP)
//    // ===============================
//
//    public double getYaw() {
//        return pinpoint.getHeading(AngleUnit.DEGREES);
//    }
//
//    public void resetGyro(double heading) {
//        pinpoint.setHeading(heading, AngleUnit.DEGREES);
//    }
//}
