package org.firstinspires.ftc.teamcode.robotSubSystems.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitUtils.PID;
import org.firstinspires.ftc.teamcode.robotData.Constants;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class Arm {
    public static DcMotor armMotor;
    public static float pos;
    public static float currentPos;
    public static double wantedPower;
    public static final PID armPID = new PID(ArmConstants.armKp, ArmConstants.armKi, ArmConstants.armKd, ArmConstants.armKf, ArmConstants.armIZone);

    //motion magic
    public static double W_V;
    public static double A_V;
    public static double Pos_A_Rad;
    public static double W_Pos;
    public static double Magic_W_Acc;
    public static double Ps;
    public static double Pg;
    public static double Pv;
    //public static final PID Arm_V_PID = new PID(ArmConstants.Motion_Kp,ArmConstants.Motion_Ki,ArmConstants.Motion_Kd,ArmConstants.Motion_Kf,ArmConstants.Motion_I_Zone);
    public static double MotorVelocity;
    public static double Ppid;
    public static double Target_Rad_Pose;
    public static double Pos_Prev_Rad;
    public static double W_Pos_Prev;
    public static double W_V_Prev;
    public static double A_V_Prev;

    public static void init(HardwareMap hardwareMap, String name) {
        armMotor = hardwareMap.get(DcMotor.class, name);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //arm positions switch case to change target pos based on input from controller
    public static void operate(ArmStates state, Gamepad gamepad1, Gamepad gamepad2) {
        switch (state) {
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
            case STACK:
                pos = ArmConstants.stackPose;
                break;
            case TRAVEL:
                pos = ArmConstants.travelPos;
                break;
            case OVERRIDE:
                pos += -gamepad1.right_stick_y * ArmConstants.overrideFactor;
                break;
        }
        //convert target pos in ticks to target pos in radians
        Target_Rad_Pose = (pos - ArmConstants.Magic_Arm_Zero) * ArmConstants.Magic_Click_to_rad;
        //convert currentPos in ticks to currentPos in rad
        currentPos = armMotor.getCurrentPosition();
        Pos_A_Rad = (currentPos - ArmConstants.Magic_Arm_Zero) * ArmConstants.Magic_Click_to_rad;

        //calculate error (distance from target to actual)
        final double pos_error = Target_Rad_Pose - Pos_A_Rad;

        //calculate acceleration to max velocity
        final double v_accel = Math.abs(W_V_Prev) + ArmConstants.Magic_Acc * Constants.teleopCycleTime;

        //calculate deacceleration to stop
        final double v_decel = Math.sqrt(2 * ArmConstants.Magic_Acc * Math.abs(Target_Rad_Pose - Pos_A_Rad));

        //calculate the wanted velocity by choosing the smallest number variable from the acceleration, deacceleration and max velocity
        W_V = Math.min(Math.min(v_accel, ArmConstants.Magic_Velocity_Max), v_decel);
        W_V = W_V * Math.signum(pos_error);

        //calculate wanted acceleration by deviding the change in velocity at new code cycle by the code cycle time to get the value of the needed acceleration at each new code cycle
        Magic_W_Acc = (W_V - W_V_Prev) / Constants.teleopCycleTime;

        //calculate wanted pos at each new cycle of the code by
        W_Pos = W_Pos_Prev + W_V_Prev * Constants.teleopCycleTime + (0.5 * Magic_W_Acc) * Math.pow(Constants.teleopCycleTime, 2);

        //calculate needed power to overcome gravity turque by multiplying the gravitational force on the arm at horizontal pos by the sin of the arm's distance from its vertical pos (Pos_A_Rad)
        Pg = -ArmConstants.MagicPg0 * Math.sin(Pos_A_Rad);
        //
        Pv = W_V * ArmConstants.Magic_Kv;
        if (Math.abs(Target_Rad_Pose - Pos_A_Rad) < 0.05) {
            Pv = 0;
            Ppid = ArmConstants.Magic_Kp * (pos_error);
            W_Pos = Target_Rad_Pose;
        } else
            Ppid = ArmConstants.Magic_Kp * (W_Pos - Pos_A_Rad);

        wantedPower = Pg + Pv + Ppid;

        Ps = Math.signum(wantedPower) * ArmConstants.MagicKs;
        wantedPower += Ps;

        Pos_Prev_Rad = Pos_A_Rad;
        W_Pos_Prev = W_Pos;
        W_V_Prev = W_V;
        A_V_Prev = A_V;

        GlobalData.armPower = wantedPower;
        //GlobalData.armPower = -gamepad1.right_stick_y/3;
        if (Math.abs(wantedPower)> ArmConstants.powerLimit)  {
            wantedPower = ArmConstants.powerLimit * Math.signum(wantedPower);
        }
        armMotor.setPower(GlobalData.armPower);
    }

    public static void test(Gamepad gamepad1, Telemetry telemetry) {
        currentPos = armMotor.getCurrentPosition();
        if (gamepad1.b) {
            armPID.setWanted(0);
        } else {
            armPID.setWanted(ArmConstants.armTestPos);
        }
        wantedPower = (float) armPID.update(currentPos);
        wantedPower = (float) Math.min(Math.abs(wantedPower), ArmConstants.powerLimit) * Math.signum(wantedPower);
        if (wantedPower < 0 && currentPos < 300)
            wantedPower = Math.max(wantedPower, -(currentPos / 300));
        armMotor.setPower(wantedPower);

//    currentPos = armMotor.getCurrentPosition();
//    pos += -gamepad1.right_stick_y * ArmConstants.overrideFactor;
//    armPID.setWanted(pos);
//    armMotor.setPower(armPID.update(currentPos));


        telemetry.addData("arm pose", armMotor.getCurrentPosition());
        telemetry.update();
    }
}
