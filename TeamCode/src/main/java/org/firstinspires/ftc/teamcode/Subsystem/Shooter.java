package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tele.TeleConstant;

public class Shooter {
    private final DcMotorEx shooterMotor;
    private final CRServo indexerLeft, indexerRight;

    // ---------- FINAL PIDF ----------
    private static final double kP = 250; //200
    private static final double kI = 0;
    private static final double kD = 8;
    private static final double kF = 18;

    // Motor RPM = Wheel RPM * GEAR_RATIO
    private static final double GEAR_RATIO = 1;

    private static final double RPM_TOLERANCE = 4;
    private double targetWheelRPM = 0;
    private double currentWheelRPM = 0;

    public boolean auto = false;

    // Auto-feed speed
    private static final double indexersSpeed = 1.0;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        indexerLeft = hardwareMap.get(CRServo.class, "IndexerLeft");
        indexerRight = hardwareMap.get(CRServo.class, "IndexerRight");

        // Use hardware direction reversal
        indexerLeft.setDirection(CRServo.Direction.REVERSE);
    }

    // ---------- TARGET RPM ----------
    public void setTargetRPM(double wheelRPM) {
        targetWheelRPM = wheelRPM;
    }

    public void setIdleRPM(double idleRPM){
        TeleConstant.ildeRPM = idleRPM;
    }

    // ---------- UPDATE ----------
    public void update() {
        double ticksPerRev = shooterMotor.getMotorType().getTicksPerRev();

        if (targetWheelRPM > 0) {
            double motorRPM = targetWheelRPM * GEAR_RATIO;

            // Clip motor RPM to max motor RPM
            motorRPM = Math.min(motorRPM, shooterMotor.getMotorType().getMaxRPM());

            double targetTicksPerSecond = (motorRPM / 60.0) * ticksPerRev;
            shooterMotor.setVelocity(targetTicksPerSecond);
        } else {
            shooterMotor.setPower(TeleConstant.ildeRPM);
            //shooterMotor.setPower(0);
        }

        double motorRPM = shooterMotor.getVelocity() / ticksPerRev * 60.0;
        currentWheelRPM = motorRPM / GEAR_RATIO;

        // ---------- AUTO FEED ----------
        if(auto) {
            /*
            if (targetWheelRPM > 0 && isAtTargetRPM()) {
                indexerLeft.setPower(indexersSpeed);
                indexerRight.setPower(indexersSpeed); // same power, hardware handles reversal
            }
             */
        }
        else{
            if (targetWheelRPM > 0 && isAtTargetRPM()) {
                indexerLeft.setPower(indexersSpeed);
                indexerRight.setPower(indexersSpeed); // same power, hardware handles reversal
            }
        }
    }
    public double RPMSpeed(double distance) {
        //beginning kinda inconsistent
        //double RPMSpeedShoot =  0.00143946 * Math.pow(distance, 2) + 0.0848202 * distance + 49.94298;
        //missing middle part
        //double RPMSpeedShoot = 0.00116369 * Math.pow(distance, 2) + 0.11544 * distance + 49.6868;
        //good but missing close shots
        //double RPMSpeedShoot = 0.00115217 * Math.pow(distance, 2) + 0.116927 * distance + 49.66988;
        //misses at a couple of places but really good right now
        //double RPMSpeedShoot = 0.00115374 * Math.pow(distance, 2) + 0.116037 * distance + 49.74027;
        //misses a a couple
        //double RPMSpeedShoot = 0.00130886 * Math.pow(distance, 2) + 0.0971697 * distance + 49.90263;
        //a bit better
        //double RPMSpeedShoot = 0.00119297 * Math.pow(distance, 2) + 0.110161 * distance + 49.8656;
        double RPMSpeedShoot = 0.00134103 * Math.pow(distance, 2) + 0.0943011 * distance + 49.84634;
        return RPMSpeedShoot;
    }


    // ---------- STOP ----------
    public void stopShooter() {
        targetWheelRPM = 0;
        //shooterMotor.setPower(0.3);
        indexerLeft.setPower(-1);
        indexerRight.setPower(-1);
    }

    public void autoShooter(){
        targetWheelRPM = 0;
    }

    // ---------- INDEXER ----------
    public void setIndexerPower(double power) {
        indexerLeft.setPower(power);
        indexerRight.setPower(power); // same power
    }

    // ---------- STATUS ----------
    public boolean isAtTargetRPM() {
        return Math.abs(targetWheelRPM - currentWheelRPM) <= RPM_TOLERANCE;
    }

    public double getTargetRPM() {
        return targetWheelRPM;
    }

    public double getCurrentRPM() {
        return currentWheelRPM;
    }

    public double RPMDiff() {
        return Math.abs(targetWheelRPM - currentWheelRPM);
    }
}
