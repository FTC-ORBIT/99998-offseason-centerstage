package org.firstinspires.ftc.teamcode.robotSubSystems.climb;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Climb {
    public static Servo servo;
    public static Servo servo2;
    public static float pos;

    public static void init(HardwareMap hardwareMap, String name, String name2){
        servo = hardwareMap.get(Servo.class,name);
        servo2 = hardwareMap.get(Servo.class,name2);
    }

    public static void operate(ClimbStates state){
        switch (state){
            case GROUND:
                pos = ClimbConstants.groundPose;
                break;
            case CLIMB:
                pos = ClimbConstants.climbPose;
                break;
        }
        servo.setPosition(pos);
        servo2.setPosition(pos);
    }
}
