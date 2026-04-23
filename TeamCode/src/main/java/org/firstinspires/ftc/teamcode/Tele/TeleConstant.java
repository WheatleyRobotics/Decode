package org.firstinspires.ftc.teamcode.Tele;

import com.pedropathing.geometry.Pose;

public class TeleConstant {
    //auto
    public static final double FirstBallBlueCloseShotRPM = 58;
    public static final double SecondBallBlueCloseShotRPM = 58;
    public static final double ThirdBallBlueCloseShotRPM = 58;

    public static final double FirstBallRedCloseShotRPM = 56; //64.5
    public static final double SecondBallRedCloseShotRPM = 56; //68.8

    public static final double startingHoodPos = 0.29; //0.39 // 0.31 is the new pos
    public static double ildeRPM = 0.3;

    public static double GyroOffsets = 40; //40

    public static final double bumperUpRPM = 50; //52
    public static final double bumperUpTeleRPM = 50;
    public static final double bumperUpGyro = 130; // left is - right is +
    //public static final double bumperUpHoodPos = 0.58; //0.59
    public static final double bumperUpOffset = 0.03; //0.06
    public static final double bumperUpTeleOffset = 0.02;
    public static final double bumperUpIdleRPM = 0.3;

    public static final double closeShotRPM = 57; //65
    public static final double closeShotGyro = 130; // left is - right is +
    //public static final double closeShotHoodPos = 0.7;
    public static final double closeShotOffset = 0.34; // 0.35
    public static final double closeShotIdleRPM = 0.3;
    public static final double closeShotX = 80;
    public static final double closeShotY = 101;

    public static final double midShotRPM = 61.5; //65
    public static final double midShotGyro = 130; //138
    //public static final double midShotHoodPos = 0.77;
    public static final double midShotOffset = 0.37; //0.38
    public static final double midIdleRPM = 0.55;

    public static final double farShotRPM = 83; //80
    public static final double farShotGyro = 135;
    //public static final double farShotHoodPos = 0.77;
    public static final double farShotOffset = 0.37; //38
    public static final double farIdleRPM = 0.6;

    public static final double robotVelocityMultiplier = 3;
    public static final double turnVelocityMultiplier = 0.5;

    public static final double outakeTime = 0.09;
    public static Pose startingPoseAfterAuto = null;
}
