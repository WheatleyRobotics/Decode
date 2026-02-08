package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class Shooter {
    //private Intake intake;

    private final DcMotor shooterMotor;
    private final CRServo indexerLeft, indexerRight;

    // ---------- PID CONSTANTS (TUNE THESE) ----------
    private static final double kP = 0.0019; //12
    private static final double kI = 0; //0
    private static final double kD = 0.05; // 0.002
    private static final double kF = 12; //0.4

    // ---------- ENCODER CONSTANTS ----------
    private static final double TICKS_PER_REV = 537.7; // goBILDA 5202
    // 5203 = 312

    // ---------- RPM CONTROL ----------
    private static final double RPM_TOLERANCE = 20; //25  acceptable error

    private double targetRPM = 0;
    private double integral = 0;
    private double lastError = 0;

    private int lastPosition = 0;
    private double lastTime = 0;

    private double currentRPM = 0;

    private static final double indexersSpeed = 1;

    private final ElapsedTime timer = new ElapsedTime();

    public Shooter(HardwareMap hardwareMap) {
        //Shooter Calibration
        shooterMotor = hardwareMap.get(DcMotor.class, "Shooter");
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Indexer Calibration
        indexerLeft = hardwareMap.get(CRServo.class, "IndexerLeft");
        indexerRight = hardwareMap.get(CRServo.class, "IndexerRight");
        //indexerRight.setDirection(DcMotorSimple.Direction.REVERSE);
        indexerLeft.setDirection(CRServo.Direction.REVERSE);

        //Timer Calibration
        timer.reset();
        lastTime = timer.seconds();
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    public boolean isAtTargetRPM() {
        return Math.abs(targetRPM - currentRPM) <= RPM_TOLERANCE;
    }

    public void update() {
        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) return;

        int currentPosition = shooterMotor.getCurrentPosition();
        int deltaTicks = currentPosition - lastPosition;

        currentRPM = (deltaTicks / TICKS_PER_REV) / deltaTime * 60.0;

        double error = targetRPM - currentRPM;

        integral += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;

        double power = (kP * error) + (kI * integral) + (kD * derivative) + (kF * targetRPM);

        power = Math.max(0.0, Math.min(1.0, power));
        shooterMotor.setPower(power);

        // ---------- AUTO FEED ----------

        if (targetRPM > 0 && isAtTargetRPM()) {
            indexerLeft.setPower(indexersSpeed);
            indexerRight.setPower(indexersSpeed);
        }

        /*
        else {
            indexerLeft.setPower(0);
            indexerRight.setPower(0);
        }
        */

        lastError = error;
        lastPosition = currentPosition;
        lastTime = currentTime;
    }

    public void setIndexerPower(double setIndexersSpeed){
        indexerLeft.setPower(setIndexersSpeed);
        indexerRight.setPower(setIndexersSpeed);
    }

    public void stopShooter() {
        targetRPM = 0;
        shooterMotor.setPower(0.3);
        indexerLeft.setPower(-1);
        indexerRight.setPower(-1);
        integral = 0;
    }

    // ---------- TELEMETRY ----------
    public double RPMDiff(){
        return Math.abs(targetRPM - currentRPM);
    }

    public double getTargetRPM(){
        return targetRPM;
    }

    public double getCurrentRPM(){
        return currentRPM;
    }

    public double getPose(){
        return shooterMotor.getCurrentPosition();
    }
}