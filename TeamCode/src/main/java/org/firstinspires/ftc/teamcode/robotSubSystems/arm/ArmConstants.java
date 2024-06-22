package org.firstinspires.ftc.teamcode.robotSubSystems.arm;


import com.acmerobotics.dashboard.config.Config;

@Config
public  class ArmConstants {
    public static final float groundPose = 0;
    public static final float travelPos = 100;
    public static final float minPose = 2100;
    public static final float lowPose = 1800;
    public static final float midPose = 1600;
    public static final float stackPose = 0; // TODO find the pos.
    public static final float climbPose = 0; // TODO find the pos.
    public static double armTestPos = 400;
    public static final float  overrideFactor = 10;
    public static  double armKp = 0.01f;
    public static  double armKi = 0.000f;
    public static  double armKd = 0.5f;
    public static  double  armKf = 0;
    public static  double armIZone =50f;
    public static double powerLimit = 0.75;
}
