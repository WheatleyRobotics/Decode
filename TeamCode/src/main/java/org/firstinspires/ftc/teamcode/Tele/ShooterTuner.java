package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class ShooterTuner extends OpMode {
    private DcMotorEx shooterMotor;

    private double highVelocity = 225;
    private double lowVelocity = 130;

    double curTargetVelocity = highVelocity;

    private double P = 0;
    private double F = 0;

    double[] stepSizes = {10, 1, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init(){
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop(){
        //get all uor game pad commands
        //set target velocity
        //update telemetry

        if(gamepad1.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            }
            else{
                curTargetVelocity = highVelocity;
            }
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasReleased()){
            P += stepSizes[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        //set new PIDF coefficents
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set velocity
        shooterMotor.setVelocity(curTargetVelocity);

        double currVelocity = shooterMotor.getVelocity();
        double error = curTargetVelocity - currVelocity;

        telemetry.addData("Target Velocity: ", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", currVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------------------------");
        telemetry.addData("Tuning P: ", "%4f (Dpad U/D)", P);
        telemetry.addData("Tuning F: ", "%4f (Dpad L/R)", F);
        telemetry.addData("Step Size: ", "%4f (B Button)", stepSizes[stepIndex]);
    }
}
