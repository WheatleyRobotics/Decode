package org.firstinspires.ftc.teamcode.Tele;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.LimeLight;
import org.firstinspires.ftc.teamcode.Commands.AutoAim;

@TeleOp
public class LimeLightTele extends OpMode {
    private LimeLight limeLight;
    //private AutoAim autoAim;

    @Override
    public void init(){
        limeLight = new LimeLight(hardwareMap);
        //autoAim = new AutoAim(hardwareMap);
    }

    @Override
    public void loop(){
        limeLight.update();
    }
}
