package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.LimeLight;

@TeleOp
public class LimeLightTele extends OpMode {
    private LimeLight limeLight;

    @Override
    public void init() {
        limeLight = new LimeLight(hardwareMap);
    }

    @Override
    public void loop() {
        limeLight.update();
        telemetry.update();

        if (limeLight.hasValidTarget()) {
            telemetry.addData("Tx: ", limeLight.getTx());
            telemetry.addData("Ty: ", limeLight.getTy());
            telemetry.addData("Ta: ", limeLight.getTa());
            telemetry.addData("Bot Pose: ", limeLight.getBotPose());
        }
        else {
            telemetry.addLine("No April Tags In Sight");
        }
    }
}
