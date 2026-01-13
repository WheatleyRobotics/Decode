package org.firstinspires.ftc.teamcode.Tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystem.Hood;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class MainTele extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(71.7, 9, Math.toRadians(90)); //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private Hood hood;

    private int RPMSpeed;
    private int gyroPos = 90; //RED Math.toRadians(0), BLUE Math.toRadians(180), And Practice Math.toRadians(90)

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "OdometryComputer");
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x*0.8, false, Math.toRadians(gyroPos));

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());

        if(gamepad1.b){
            pinpoint.setHeading(gyroPos, AngleUnit.DEGREES);
        }

        //Shooter
        if (gamepad1.right_bumper) {
            shooter.setTargetRPM(RPMSpeed); // 175
        }
        else{
            shooter.stopShooter();
        }

        if(gamepad2.dpad_up){
            RPMSpeed = 130;
            //targetAngle = 125;
        }
        else if(gamepad2.dpad_right){
            RPMSpeed = 190;
            //targetAngle = 125;
        }
        if(gamepad2.dpad_down){
            RPMSpeed = 225;
            //targetAngle = 100;
        }

        // -------- SHOOTER --------
        //Run Indexer
        boolean shooterFeeding = shooter.getTargetRPM() > 0 && shooter.isAtTargetRPM();

        if (shooterFeeding) {
            //intake.intakeWithShoot();
            intake.intakeIn();
        }
        else if (gamepad1.left_bumper) {
            intake.intakeIn();
            //shooter.setIndexerPower(-1);
        }
        else if (gamepad1.y) {
            intake.intakeOut();
            //shooter.setIndexerPower(-1);
        }
        else{
            intake.stopIntaking();
            intake.stopTunnel();
        }

        shooter.update();

        /*
        if (gamepad1.left_bumper) {
            intake.intakeIn();
            //shooter.setIndexerPower(-1);
        }
        else if (gamepad1.x) {
            intake.intakeOut();
            //shooter.setIndexerPower(-1);
        }
        else if(gamepad1.y){
            intake.tunnelRun();
            shooter.setIndexerPower(1);
        }
        else{
            intake.stopIntaking();
            intake.stopTunnel();
        }
         */



        /*
        if (gamepad1.right_bumper) {
            shooter.setTargetRPM(210);
        } else {
            shooter.stopShooter();
            feeding = false; // reset latch
        }

        // Latch when shooter reaches speed
        if (!feeding && shooter.isAtTargetRPM()) {
            feeding = true;
        }

        // Feed while latched
        if (feeding) {
            intake.intakeWithShoot();
        }
        else if (gamepad1.left_bumper) {
            intake.intakeIn();
            shooter.setIndexerPower(-1);
        }
        else if (gamepad1.y) {
            intake.intakeOut();
            shooter.setIndexerPower(-1);
        }
        else {
            intake.stopIntaking();
            intake.stopTunnel();
        }

        shooter.update();

         */

        /*
        if(shooter.getTargetRPM() > 0 && shooter.isAtTargetRPM()) {
            intake.intakeWithTunnel();
            stopTunnnel = false;
        }
        else{
            stopTunnnel = true;
        }

        if (stopTunnnel == true){
            intake.stopTunnel();
        }

        shooter.update();

        // -------- INTAKE --------
        if (gamepad1.left_bumper) {
            intake.intakeIn();
            shooter.setIndexerPower(-1);
        }
        else if (gamepad1.y) {
            intake.intakeOut();
            shooter.setIndexerPower(-1);
        }
        else {
            intake.stopIntaking();
        }

         */

        //-------- HOOD --------
        if (gamepad2.y) {
            hood.up();
        }
        else if (gamepad2.x) {
            hood.down();
        }
        else {
            hood.stop();
        }

        // -------- TELEMETRY --------
        //Drive
        //telemetry.addData("Gyro", drivetrain.getYaw());
        //Odometry
        /*
        telemetry.addData("Y Position (in)", odometry.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(odometry.heading));
        telemetry.addData("Left Odo", odometry.leftEncoder.getCurrentPosition());
        telemetry.addData("Right Odo", odometry.rightEncoder.getCurrentPosition());

         */
        //Shooter
        telemetry.addData("Shooter Ready:", shooter.isAtTargetRPM());
        telemetry.addData("Target RPM:", shooter.getTargetRPM());
        telemetry.addData("Current RPM:", shooter.getCurrentRPM());
        telemetry.addData("RPM Difference:", shooter.RPMDiff());
        telemetry.update();
    }


}