package org.firstinspires.ftc.teamcode.Tele;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class LimeLightAutoDrive extends OpMode {
    private Limelight3A camera; //any camera here
    private LLResult llResult;
    private Follower follower;
    private boolean following = false;
    private final Pose TARGET_LOCATION = new Pose(); //Put the target location here

    @Override
    public void init() {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose()); //set your starting pose
    }

    @Override
    public void start() {
        camera.start();
    }

    @Override
    public void loop() {
        follower.update();
        llResult = camera.getLatestResult();

        //if you're not using limelight you can follow the same steps: build an offset pose, put your heading offset, and generate a path etc

        if (!following) {
            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
                            .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta())
                            .build()
            );
        }

        //This uses the aprilTag to relocalize your robot
        //You can also create a custom AprilTag fusion Localizer for the follower if you want to use this by default for all your autos
        follower.setPose(getRobotPoseFromCamera());

        if (following && !follower.isBusy()) following = false;
    }

    private Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this

        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        if (llResult != null && llResult.isValid()) {
            //Pose3D botpose = camera.getLatestResult().getBotpose_MT2();
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
                    headingRadians,
                    FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        } else {
            return null;
        }

        //return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}