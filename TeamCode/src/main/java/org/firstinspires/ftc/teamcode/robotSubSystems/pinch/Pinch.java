package org.firstinspires.ftc.teamcode.robotSubSystems.pinch;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Pinch {
    public static Servo servo;
    public static Servo servo2;
    public static float pos;
    public static float pos2;
    public static boolean leftBumper;
    public static boolean rightBumper;
    public static boolean dpad_left;
    public static boolean dpad_right;
    public static boolean firstTime = true;
    public static void init(HardwareMap hardwareMap, String name, String name2){
        servo = hardwareMap.get(Servo.class,name);
        servo2 = hardwareMap.get(Servo.class,name2);
    }
    public static void operate(PinchStates state){
        switch (state){
            case CLOSED:
                pos = PinchConstants.closedPose;
                pos2 = PinchConstants.closedPose2;
                break;
            case OPEN:
                pos = PinchConstants.openPose;
                pos2 = PinchConstants.openPose2;
                break;
            case RIGHT:
                pos = PinchConstants.openPose;
                break;
            case LEFT:
                pos2 = PinchConstants.openPose2;
                break;
            case INTAKELEFT:
                pos2 = PinchConstants.closedPose2;
                break;
            case INTAKERIGHT:
                pos = PinchConstants.closedPose;
                break;
        }
        servo.setPosition(pos);
        servo2.setPosition(pos2);
    }
    public static void test(Gamepad gamepad1, Telemetry telemetry){
//        if (gamepad1.left_bumper && !leftBumper){
//            pos += 0.05;
//            pos2 += 0.05;
//        } else if (gamepad1.right_bumper && !rightBumper) {
//            pos -=0.05;
//            pos2 -= 0.05;
//        } else if (gamepad1.dpad_left && !dpad_left) {
//            pos += 0.01;
//            pos2 += 0.01;
//        } else if (gamepad1.dpad_right && !dpad_right) {
//            pos -= 0.01;
//            pos2 -= 0.01;
//        }
//        if (firstTime){
//            pos2 = 0.5f;
//            pos = 0.5f;
//            firstTime = false;
//        }
        if (gamepad1.left_bumper){
            pos = PinchConstants.closedPose;
            pos2 = PinchConstants.closedPose2;
        } else if (gamepad1.right_bumper) {
            pos = PinchConstants.openPose;
            pos2 = PinchConstants.openPose2;
        }
        leftBumper = gamepad1.left_bumper;
        rightBumper = gamepad1.right_bumper;
        dpad_left = gamepad1.dpad_left;
        dpad_right = gamepad1.dpad_right;
        servo.setPosition(pos);
        servo2.setPosition(pos2);
        telemetry.addData("pos",servo.getPosition());
        telemetry.update();
    }
}
