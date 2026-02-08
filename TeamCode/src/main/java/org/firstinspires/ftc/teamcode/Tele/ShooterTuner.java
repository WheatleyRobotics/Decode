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
    private double midVelocity =  192;
    private double lowVelocity = 130;

    double curTargetVelocity = midVelocity;

    private double P = 0;
    private double I = 0;
    private double D = 0;
    private double F = 0;

    double[] stepSizes = {10, 1, 0.1, 0.01, 0.001, 0.0001, 0.00001};

    int stepIndex = 1;

    @Override
    public void init(){
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop(){
        //get all uor game pad commands
        //set target velocity
        //update telemetry

        if(gamepad1.leftBumperWasPressed()){
            if(curTargetVelocity == lowVelocity){
                curTargetVelocity = highVelocity;
            }
            else if(curTargetVelocity == midVelocity){
                curTargetVelocity = lowVelocity;
            }
            else{
                curTargetVelocity = midVelocity;
            }
        }

        if(gamepad1.rightBumperWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadUpWasReleased()){
            P += stepSizes[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadLeftWasPressed()){
            I += stepSizes[stepIndex];
        }

        if(gamepad1.dpadRightWasPressed()){
            I -= stepSizes[stepIndex];
        }

        if(gamepad1.yWasPressed()){
            D += stepSizes[stepIndex];
        }

        if(gamepad1.aWasPressed()){
            D -= stepSizes[stepIndex];
        }

        if(gamepad1.xWasPressed()){
            F += stepSizes[stepIndex];
        }

        if(gamepad1.bWasPressed()){
            F -= stepSizes[stepIndex];
        }

        //set new PIDF coefficents
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
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
        telemetry.addData("Tuning I: ", "%4f (Dpad L/R)", I);
        telemetry.addData("Tuning D: ", "%4f (Button Y/A)", D);
        telemetry.addData("Tuning F: ", "%4f (Button X/B)", F);
        telemetry.addData("Step Size: ", "%4f (Right Bumper)", stepSizes[stepIndex]);
    }
}
