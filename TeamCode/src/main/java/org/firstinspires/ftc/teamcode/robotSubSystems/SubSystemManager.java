package org.firstinspires.ftc.teamcode.robotSubSystems;

import static org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrain.Drivetrain.motors;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveByAprilTags.Camera;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.positionTracker.PoseStorage;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.ArmStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;

public class SubSystemManager {

    public static RobotState lastState = RobotState.TRAVEL;

    public static RobotState wanted = RobotState.TRAVEL;
    public static ArmStates armState  = ArmStates.GROUND;
    public static boolean  armToggleButton = false;



    private static RobotState getState(Gamepad gamepad) {
        if (gamepad.b || gamepad.a || gamepad.x || gamepad.y || gamepad.back || gamepad.dpad_up){
            armToggleButton = false;
        }
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
               :gamepad.x ? RobotState.LOW : gamepad.y ? RobotState.MID : gamepad.back ? RobotState.CLIMB:gamepad.dpad_up ? RobotState.STACK : lastState;
    }

    private static RobotState getStateFromWantedAndCurrent(RobotState stateFromDriver) {

        switch (stateFromDriver) {
            case TRAVEL:
                break;
            case INTAKE:
                break;
            case LOW:
                break;
            case MID:
                break;
            case CLIMB:
                break;
            case STACK:
                break;

        }
        return stateFromDriver;
    }

    public static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2) {
        wanted = getState(gamepad1);




        switch (wanted) {
            case TRAVEL:
              if (!armToggleButton){
                  armState = ArmStates.GROUND;
              }
                break;
            case INTAKE:
                if (!armToggleButton) {
                    armState = ArmStates.GROUND;
                }
                break;
            case STACK:
                if (!armToggleButton) {
                    armState = ArmStates.STACK;
                }
                break;
            case LOW:
                if (!armToggleButton) {
                    armState = ArmStates.LOW;
                }
                break;
            case MID:
                if (!armToggleButton) {
                    armState = ArmStates.MID;
                }
                break;
            case CLIMB:
                if (!armToggleButton) {
                    armState = ArmStates.GROUND;
                }
                break;
        }
        if (gamepad1.right_stick_y != 0) {
            armState = ArmStates.OVERRIDE;
            armToggleButton = true;
        }
        Arm.operate(armState, gamepad1, gamepad2);



        lastState = wanted;

        if (gamepad1.dpad_down) OrbitGyro.resetGyro();
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("Robot current state ", SubSystemManager.wanted);
        telemetry.addData("Robot last state", SubSystemManager.lastState);
        if (Camera.targetFound){
            telemetry.addData("tag has found",Camera.desiredTag.id);
        }else if (Camera.skippingTagTelemetry){
            telemetry.addData("other tag ha found, skipping",Camera.desiredTag.id);
        }else {
            telemetry.addLine("tag not in the lib");
        }
        telemetry.addData("armPose", Arm.currectPos);
//        telemetry.addData("X",PoseStorage.currentPose.getX());
//        telemetry.addData("Y",PoseStorage.currentPose.getY());
        telemetry.addData("gyro", Math.toDegrees(PoseStorage.currentPose.getHeading()));
        telemetry.addData("lastAngle", OrbitGyro.lastAngle);
        telemetry.addData("currentTime", GlobalData.currentTime);
        telemetry.addData("lastTime", GlobalData.lastTime);
        telemetry.addData("deltaTime",GlobalData.deltaTime);
        telemetry.addData("lf power" , motors[0].getPower());
        telemetry.addData("rf power" , motors[1].getPower());
        telemetry.addData("lb power" , motors[2].getPower());
        telemetry.addData("rb power" , motors[3].getPower());
//        telemetry.addData("distance in inch", OrbitDistanceSensor.getDistance());
//        telemetry.addData("color", OrbitColorSensor.hasGamePiece());
//        telemetry.addData("magnetic press?", MagneticSensor.getState());
//        telemetry.addData("touchSensor press?", TouchSensor.getState());
//        telemetry.addData("potentiometer", Potentiometer.getVolt());
    }
}

//dani yalechan!
// yoel yalechan!