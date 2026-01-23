package org.firstinspires.ftc.teamcode.Tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

//import org.firstinspires.ftc.teamcode.Subsystem.Hood;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.LimeLight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Commands.AutoAim;

@Configurable
@TeleOp
public class PracticeTele extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(71.7, 9, Math.toRadians(90)); //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;
    private Drivetrain drivetrain;
    private LimeLight limeLight;
    private Shooter shooter;
    private Intake intake;
    //private Hood hood;
    private AutoAim autoAim;

    private int gyroPos = 90; //RED: 0, BLUE: 180, And Practice: 90
    private double gyroShootPos = 100;
    private boolean lastRightTrigger = false;

    private double endGameStart;
    private boolean  isEndGame;

    private int RPMSpeed;
    private double hoodPos;

    double turnInput = 0;
    boolean autoAimActive = false;

    //private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shooter = new Shooter(hardwareMap);
        //limeLight = new LimeLight(hardwareMap);
        intake = new Intake(hardwareMap);
        //hood = new Hood(hardwareMap);
        autoAim = new AutoAim(hardwareMap);

        //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "OdometryComputer");

        ///hood.setRange(0, 0.9);
        //hood.resetServo();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        //hood.setHoodPos(0);
        follower.startTeleopDrive();
        endGameStart = getRuntime() + 90;
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        limeLight.update();

        //follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x*0.8, false, Math.toRadians(gyroPos));

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());

        // Activate auto aim
        // Reset gyro
        if (gamepad1.b) {
            autoAim.resetGyro(gyroPos);
        }

        boolean rightTriggerPressed = gamepad1.right_trigger > 0.8;

        // Trigger just pressed
        if (rightTriggerPressed && !lastRightTrigger) {
            autoAimActive = true;
        }

        // Save trigger state
        lastRightTrigger = rightTriggerPressed;


        // Determine turn power
        if (autoAimActive) {
            turnInput = autoAim.getTurnPower(gyroShootPos);

            if (autoAim.isAimed(gyroShootPos)) {
                autoAimActive = false;
            }
        } else {
            turnInput = -gamepad1.right_stick_x * 0.8;
        }

        // Drive (ONLY ONCE)
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turnInput,
                false,
                Math.toRadians(gyroPos)
        );

        //Shooter
        if (gamepad1.right_bumper) {
            shooter.setTargetRPM(RPMSpeed); // 175
        }
        else{
            shooter.stopShooter();
        }

        if(gamepad1.dpad_up){
            RPMSpeed = 130;
            //hood.setHoodPos(0.6);
            gyroShootPos = 142;
            //targetAngle = 125;
        }
        else if(gamepad1.dpad_right){
            RPMSpeed = 192;
            //hood.setHoodPos(0.78);
            gyroShootPos = 47;
            //targetAngle = 125;
        }
        else if(gamepad1.dpad_down){
            RPMSpeed = 225;
            //hood.setHoodPos(0.8);
            gyroShootPos = 47;
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

        //-------- HOOD --------

        if(gamepad2.y){
            //hood.manualUp();
        }
        else if(gamepad2.a){
            //hood.manualDown();
        }

        if(endGameStart >= getRuntime() && !isEndGame){
            gamepad1.rumbleBlips(5);
            gamepad2.rumbleBlips(5);
            isEndGame = true;
        }

        /*
        else {
            hood.stop();
        }
         */

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
        //Hood Servo
        //telemetry.addData("Servo Pos: ", hood.getServoPos());
        //gyro
        telemetry.addData("Pinpoint Yaw (deg)", autoAim.getYaw());
        telemetry.addData("Target Yaw (deg)", gyroShootPos);
        telemetry.addData("Auto Aim Active", autoAimActive);
        telemetry.update();
    }


}