package org.firstinspires.ftc.teamcode.Tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Shooter PIDF Tuner Graph", group = "Tuning")
@Configurable
public class ShooterPIDFTunerGraph extends OpMode {

    private DcMotorEx shooterMotor;
    private ElapsedTime timer;

    // ===== EDIT THESE IN PANELS =====
    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;

    public static double targetRPM = 130; // example: 130, 192, 255

    private PIDFCoefficients lastPIDF = new PIDFCoefficients(P, I, D, F);

    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        timer = new ElapsedTime();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");

        // ===== YOUR ORIGINAL SETUP =====
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
    }

    @Override
    public void loop() {

        // Update PIDF live from Panels
        if (lastPIDF.p != P || lastPIDF.i != I ||
                lastPIDF.d != D || lastPIDF.f != F) {

            shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
            lastPIDF = new PIDFCoefficients(P, I, D, F);
        }

        double ticksPerRev = shooterMotor.getMotorType().getTicksPerRev();
        double targetTicksPerSecond = (targetRPM / 60.0) * ticksPerRev;

        shooterMotor.setVelocity(targetTicksPerSecond);

        double currentRPM = shooterMotor.getVelocity() / ticksPerRev * 60.0;

        // ===== Telemetry =====
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        telemetry.update();

        // ===== Panels Graph =====
        // Use time for X-axis
        double t = timer.seconds();

        panelsTelemetry.addData("TargetRPM", targetRPM);
        panelsTelemetry.addData("CurrentRPM", currentRPM);
        panelsTelemetry.addData("Time", t); // optional, useful for plotting vs time

        // Example: also show difference/error
        panelsTelemetry.addData("RPMError", targetRPM - currentRPM);

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
    }
}
