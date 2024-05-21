package org.firstinspires.ftc.teamcode.robotSubSystems.plane;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane {
    public static Servo servo;
    public static Servo servo2;
    public static float pos;
    public static float pos2;
    public static boolean leftBumper;
    public static boolean rightBumper;
    public static boolean dpad_left;
    public static boolean dpad_right;
    public static void init(HardwareMap hardwareMap, String name, String name2){
        servo = hardwareMap.get(Servo.class,name);
        servo2 = hardwareMap.get(Servo.class,name2);
    }

    public static void operate(PlaneStates state){
        switch (state){
            case STOP:
                pos = PlaneConstants.stopPose;
                pos2 = PlaneConstants.systemAngle;
                break;
            case THROW:
                pos = PlaneConstants.throwPose;
                pos2 = PlaneConstants.systemAngle;
                break;
        }
        servo.setPosition(pos);
        servo2.setPosition(pos2);
    }
    public static void test(Gamepad gamepad1){
        if (gamepad1.left_bumper && !leftBumper){
            pos += 0.05;
        } else if (gamepad1.right_bumper && !rightBumper) {
            pos -=0.05;
        } else if (gamepad1.dpad_left && !dpad_left) {
            pos += 0.01;
        } else if (gamepad1.dpad_right && !dpad_right) {
            pos -= 0.01;
        }
        leftBumper = gamepad1.left_bumper;
        rightBumper = gamepad1.right_bumper;
        dpad_left = gamepad1.dpad_left;
        dpad_right = gamepad1.dpad_right;
        servo.setPosition(pos);

    }
}
