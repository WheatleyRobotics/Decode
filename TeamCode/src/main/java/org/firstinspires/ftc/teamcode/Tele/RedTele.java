package org.firstinspires.ftc.teamcode.Tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Hood;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Commands.AutoAim;

@Configurable
@TeleOp
public class RedTele extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(71.7, 9, Math.toRadians(90));
    private TelemetryManager telemetryM;
    private Shooter shooter;
    private Intake intake;
    private Hood hood;
    private AutoAim autoAim;

    private int gyroPos = 0; //RED: 0, BLUE: 180, And Practice: 90
    private double gyroShootPos = 100;
    private boolean lastRightTrigger = false;

    private int RPMSpeed;
    private double hoodPos;

    double turnInput = 0;
    boolean autoAimActive = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);
        autoAim = new AutoAim(hardwareMap);

        hood.setRange(0, 0.9);
    }

    @Override
    public void start() {
        hood.setHoodPos(0);
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());

        if (gamepad1.b) {
            autoAim.resetGyro(gyroPos);
        }

        boolean rightTriggerPressed = gamepad1.right_trigger > 0.8;

        if (rightTriggerPressed && !lastRightTrigger) {
            autoAimActive = true;
        }

        lastRightTrigger = rightTriggerPressed;

        // -------- FIX STARTS HERE --------
        // If driver tries to turn, cancel auto aim
        if (Math.abs(gamepad1.right_stick_x) > 0.08) {
            autoAimActive = false;
        }
        // -------- FIX ENDS HERE --------

        if (autoAimActive) {
            turnInput = autoAim.getTurnPower(gyroShootPos);

            if (autoAim.isAimed(gyroShootPos)) {
                autoAimActive = false;
            }
        } else {
            turnInput = -gamepad1.right_stick_x * 0.8;
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turnInput,
                false,
                Math.toRadians(gyroPos)
        );

        if (gamepad1.right_bumper) {
            shooter.setTargetRPM(RPMSpeed);
        }
        else{
            shooter.stopShooter();
        }

        if(gamepad1.dpad_up){
            RPMSpeed = 20;
            hood.setHoodPos(0.6);
            gyroShootPos = 130;
        }
        else if(gamepad1.dpad_right){
            RPMSpeed = 35;
            hood.setHoodPos(0.78);
            gyroShootPos = 137;
        }
        else if(gamepad1.dpad_down){
            RPMSpeed = 50;
            hood.setHoodPos(0.8);
            gyroShootPos = 100;
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
        else{
            intake.stopIntaking();
            intake.stopTunnel();
        }

        shooter.update();

        if(gamepad2.y){
            hood.manualUp();
        }
        else if(gamepad2.a){
            hood.manualDown();
        }

        telemetry.addData("Shooter Ready:", shooter.isAtTargetRPM());
        telemetry.addData("Target RPM:", shooter.getTargetRPM());
        telemetry.addData("Current RPM:", shooter.getCurrentRPM());
        telemetry.addData("RPM Difference:", shooter.RPMDiff());
        telemetry.addData("Servo Pos: ", hood.getServoPos());
        telemetry.addData("Pinpoint Yaw (deg)", autoAim.getYaw());
        telemetry.addData("Target Yaw (deg)", gyroShootPos);
        telemetry.addData("Auto Aim Active", autoAimActive);
        telemetry.update();
    }
}
