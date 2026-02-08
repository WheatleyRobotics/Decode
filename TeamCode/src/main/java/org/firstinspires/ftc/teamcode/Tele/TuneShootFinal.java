package org.firstinspires.ftc.teamcode.Tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Tune Shoot W Intake", group = "Tuning")
@Configurable
public class TuneShootFinal extends OpMode {
    private DcMotorEx shooterMotor;
    private DcMotor intakeMotor;
    private DcMotor tunnelMotor;
    private CRServo indexerLeft;
    private CRServo indexerRight;

    private ElapsedTime timer;

    // ===== EDIT THESE IN PANELS =====
    public static double P = 0.0;        // Proportional
    public static double I = 0.0;        // Integral
    public static double D = 0.0;        // Derivative
    public static double F = 0.0;        // Feedforward

    public static double targetRPM = 130; // keep this at the bottom

    private PIDFCoefficients lastPIDF = new PIDFCoefficients(P, I, D, F);

    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        timer = new ElapsedTime();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        tunnelMotor = hardwareMap.get(DcMotor.class, "Tunnel");
        indexerLeft = hardwareMap.get(CRServo.class, "IndexerLeft");
        indexerRight = hardwareMap.get(CRServo.class, "IndexerRight");

        // ===== ORIGINAL MOTOR SETUP =====
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        indexerLeft.setDirection(CRServo.Direction.REVERSE);

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

        // Convert target RPM to encoder ticks per second
        double ticksPerRev = shooterMotor.getMotorType().getTicksPerRev();
        double targetTicksPerSecond = (targetRPM / 60.0) * ticksPerRev;

        shooterMotor.setVelocity(targetTicksPerSecond);

        double currentRPM = shooterMotor.getVelocity() / ticksPerRev * 60.0;

        // ===== FEEDER SYSTEM ALWAYS RUNNING =====
        intakeMotor.setPower(1.0);       // Intake spinning forward
        tunnelMotor.setPower(0.8);       // Tunnel spinning forward
        indexerLeft.setPower(1.0);       // Indexer left forward
        indexerRight.setPower(1.0);      // Indexer right forward

        // ===== Standard Telemetry =====
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("RPM Error", targetRPM - currentRPM);
        telemetry.update();

        // ===== Panels Graph Telemetry =====
        double t = timer.seconds();

        panelsTelemetry.addData("P", P);
        panelsTelemetry.addData("I", I);
        panelsTelemetry.addData("D", D);
        panelsTelemetry.addData("F", F);
        panelsTelemetry.addData("TargetRPM", targetRPM);
        panelsTelemetry.addData("CurrentRPM", currentRPM);
        panelsTelemetry.addData("RPMError", targetRPM - currentRPM);
        panelsTelemetry.addData("Time", t); // optional for graphing vs time
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
        intakeMotor.setPower(0);
        tunnelMotor.setPower(0);
        indexerLeft.setPower(0);
        indexerRight.setPower(0);
    }
}
