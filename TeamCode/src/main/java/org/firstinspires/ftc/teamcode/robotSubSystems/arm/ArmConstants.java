package org.firstinspires.ftc.teamcode.robotSubSystems.arm;


import com.acmerobotics.dashboard.config.Config;

@Config
public  class ArmConstants {
    public static final float groundPose = 10;
    public static final float travelPos = 150;
    public static final float minPose = 2050;
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
    public static double powerLimit = 1.0d;
    public static double MagicKs = 0.02;
    public static double MagicPg0 = 0.18;
    public static double Magic_Arm_Zero = 1251;
    public static double Magic_Kv=0.23d;
    public static float Magic_Kp = 0.25f;
    public static double Magic_Velocity_Max = 3d;
    public static double Magic_Acc = 2d;
    public static double Click_Per_Rev = 3919.025d;
    public static double Magic_Click_to_rad =  (2*Math.PI)/Click_Per_Rev;
}
