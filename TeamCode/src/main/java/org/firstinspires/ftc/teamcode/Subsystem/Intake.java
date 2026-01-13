package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor intakeMotor;
    private final DcMotor tunnelMotor;

    private static final double INTAKE_SPEED = 1;
    private static final double INTAKE_SHOOT_SPEED = 0.5;
    private static final double TUNNEL_SPEED = 1;
    private static final double TUNNEL_SHOOT_SPEED = 0.8;

    public Intake(HardwareMap hardwareMap) {
        //Intake Config
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Tunnel Config
        tunnelMotor = hardwareMap.get(DcMotor.class, "Tunnel");

        tunnelMotor.setDirection(DcMotor.Direction.FORWARD);
        tunnelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intakeWithShoot(){
        intakeMotor.setPower(INTAKE_SHOOT_SPEED);
        tunnelMotor.setPower(TUNNEL_SPEED);
    }

    public void intakeIn() {
        intakeMotor.setPower(INTAKE_SPEED);
        tunnelMotor.setPower(TUNNEL_SPEED);
    }

    public void intakeOut(){
        intakeMotor.setPower(-INTAKE_SPEED);
        tunnelMotor.setPower(-TUNNEL_SPEED);
    }


    public void stopIntaking() {
        intakeMotor.setPower(0);
    }

    public void tunnelRun(){
        tunnelMotor.setPower(TUNNEL_SPEED);
    }

    public void stopTunnel(){
        tunnelMotor.setPower(0);
    }
}