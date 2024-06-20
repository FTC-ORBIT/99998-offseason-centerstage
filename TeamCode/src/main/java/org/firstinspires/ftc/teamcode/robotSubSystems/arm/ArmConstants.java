package org.firstinspires.ftc.teamcode.robotSubSystems.arm;


import com.acmerobotics.dashboard.config.Config;

@Config
public  class ArmConstants {
    public static final float groundPose = 0;
    public static final float minPose = 2100;
    public static final float lowPose = 1800;
    public static final float midPose = 1553;
    public static final float climbPose = 0; // TODO find the pos.
    public static double armTestPos = 1100;
    public static final float  overrideFactor = 20;
    public static  final float armKp = 0.006f;
    public static  final float armKi = 0.0001f;
    public static  final float armKd = 0.001f;
    public static  final float armKf = 0;
    public static  final float armIZone =50f;
}
