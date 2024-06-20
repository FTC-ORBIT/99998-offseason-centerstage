package org.firstinspires.ftc.teamcode.robotSubSystems.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.PID;

public class Arm {
    public static DcMotor armMotor;
    public static float pos;
    public static float currentPos;
    public static float zeroPose;
    public static final PID armPID = new PID(ArmConstants.armKp,ArmConstants.armKi,ArmConstants.armKd,ArmConstants.armKf,ArmConstants.armIZone);
public static void init(HardwareMap hardwareMap, String name){
    armMotor = hardwareMap.get(DcMotor.class,name);

    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
}

public static void operate(ArmStates state, Gamepad gamepad1, Gamepad gamepad2){
     switch (state){
         case GROUND:
             pos = ArmConstants.groundPose;
             break;
         case MIN:
             pos = ArmConstants.minPose;
             break;
         case LOW:
             pos = ArmConstants.lowPose;
             break;
         case MID:
             pos = ArmConstants.midPose;
             break;
         case CLIMB:
             pos = ArmConstants.climbPose;
             break;
         case OVERRIDE:
             pos += -gamepad1.right_stick_y * ArmConstants.overrideFactor;
             break;
        }
        currentPos = armMotor.getCurrentPosition() - zeroPose;
        armPID.setWanted(pos);
        armMotor.setPower(armPID.update(currentPos));

        if (gamepad2.left_bumper){
            zeroPose = armMotor.getCurrentPosition();
        }
    }
    public static void test(Gamepad gamepad1, Telemetry telemetry){
//    currentPos = armMotor.getCurrentPosition();
//    if (gamepad1.b){
        armPID.setWanted(ArmConstants.armTestPos);
//    }else {
        armPID.setWanted(0);
//    }
//    armMotor.setPower(armPID.update(currentPos));

    currentPos = armMotor.getCurrentPosition();
    pos += -gamepad1.right_stick_y * ArmConstants.overrideFactor;
    armPID.setWanted(pos);
    armMotor.setPower(armPID.update(currentPos));

    telemetry.addData("arm pose",armMotor.getCurrentPosition());
    telemetry.update();
    }
}
