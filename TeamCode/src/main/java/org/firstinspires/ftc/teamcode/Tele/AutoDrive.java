package org.firstinspires.ftc.teamcode.Tele;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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
public class AutoDrive extends OpMode {
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

    private int gyroPos = 180; // RED: 0, BLUE: 180, PRACTICE: 90
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
        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
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
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        hood.setHoodPos(TeleConstant.startingHoodPos);
        shooter.auto = false;
        camera.start();
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

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        llResult = camera.getLatestResult();

        /*
        if(llResult != null && llResult.isValid()) {
            follower.setPose(getRobotPoseFromCamera());
        }
         */
        if(getRobotPoseFromCamera() != null){
            follower.setPose(getRobotPoseFromCamera());
            lastCurrentLimeLightPos = getRobotPoseFromCamera();
        }

        if (gamepad1.b) {
            autoAim.resetGyro(gyroPos);
        }

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false, // Robot Centric
                    Math.toRadians(gyroPos)
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false, // Robot Centric
                    Math.toRadians(gyroPos)
            );
        }

        //Automated PathFollowing
        if (gamepad1.left_trigger > 0.8) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1 || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        /*
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }
         */

        if (gamepad1.right_trigger > 0.8) {
            shooter.setTargetRPM(RPMSpeed);
        }
        else {
            shooter.stopShooter();
        }

        if (gamepad1.dpad_up) {
            RPMSpeed = TeleConstant.bumperUpRPM;
            //hood.setHoodPos(TeleConstant.bumperUpHoodPos);
            hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.bumperUpOffset);
            gyroShootPos = TeleConstant.bumperUpGyro;
            shooter.setIdleRPM(TeleConstant.bumperUpIdleRPM);
        }
        else if(gamepad1.dpad_right){
            RPMSpeed = TeleConstant.closeShotRPM;
            //hood.setHoodPos(TeleConstant.closeShotHoodPos);
            hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.closeShotOffset);
            gyroShootPos = TeleConstant.closeShotGyro;
            shooter.setIdleRPM(TeleConstant.closeShotIdleRPM);
        }
        else if (gamepad1.dpad_down) {
            RPMSpeed = TeleConstant.midShotRPM;
            //hood.setHoodPos(TeleConstant.midShotHoodPos);
            hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.midShotOffset);
            gyroShootPos = TeleConstant.midShotGyro;
            shooter.setIdleRPM(TeleConstant.midIdleRPM);
        }
        else if (gamepad1.dpad_left) {
            RPMSpeed = TeleConstant.farShotRPM;
            //hood.setHoodPos(TeleConstant.farShotHoodPos);
            hood.setHoodPos(TeleConstant.startingHoodPos + TeleConstant.farShotOffset);
            gyroShootPos = TeleConstant.farShotGyro;
            shooter.setIdleRPM(TeleConstant.farIdleRPM);
        }

        boolean shooterFeeding = shooter.getTargetRPM() > 0 && shooter.isAtTargetRPM();

        if (shooterFeeding) {
            intake.intakeIn();
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

        shooter.update();

        if (gamepad2.y) {
            hood.manualUp();
        }
        else if (gamepad2.a) {
            hood.manualDown();
        }


        telemetry.addData("Shooter Ready", shooter.isAtTargetRPM());
        telemetry.addData("Target RPM", shooter.getTargetRPM());
        telemetry.addData("Current RPM", shooter.getCurrentRPM());
        telemetry.addData("RPM Difference", shooter.RPMDiff());
        telemetry.addData("Servo Pos", hood.getServoPos());
        telemetry.addData("Pinpoint Yaw (deg)", follower.getHeading());
        telemetry.addData("Target Yaw (deg)", gyroShootPos);
        telemetry.addData("Auto Aim Active", autoAimActive);
        //telemetry.addData("Limelight Valid Target", limeLight.hasValidTarget());
        telemetry.addData("Odometry Pose", follower.getPose());
        //Panels Telemetry
        telemetryM.addData("Target RPM", RPMSpeed);
        telemetryM.addData("Current pose", follower.getPose());
        telemetryM.addData("Current RPM", shooter.getCurrentRPM());
        if(getRobotPoseFromCamera() != null){
            telemetry.addData("Limelight Pose:", getRobotPoseFromCamera());
        }
        else {
            telemetry.addData("LimelightPose", "Null");
        }

        telemetry.update();

        //telemetryM.debug("position", follower.getPose());
        telemetryM.addData("Odometry Pose", follower.getPose());
        if(getRobotPoseFromCamera() != null){
            telemetryM.addData("Limelight Pose:", getRobotPoseFromCamera());
        }
        else {
            telemetryM.addData("LimelightPose", lastCurrentLimeLightPos);
        }
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}