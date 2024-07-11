package org.firstinspires.ftc.teamcode.robotSubSystems;

import static org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrain.Drivetrain.motors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveByAprilTags.Camera;
import org.firstinspires.ftc.teamcode.OrbitUtils.Delay;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.positionTracker.PoseStorage;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.ArmStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.pinch.Pinch;
import org.firstinspires.ftc.teamcode.robotSubSystems.pinch.PinchConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.pinch.PinchStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.PlaneStates;

public class SubSystemManager {

    public static RobotState lastState = RobotState.TRAVEL;

    public static RobotState wanted = RobotState.TRAVEL;
    public static ArmStates armState  = ArmStates.GROUND;
    public static boolean  armToggleButton = false;
    public static PinchStates pinchState = PinchStates.CLOSED;
    public static Delay armDelay = new Delay(0.3f);



    private static RobotState getState(Gamepad gamepad) {
        if (gamepad.b || gamepad.a || gamepad.x || gamepad.y || gamepad.back || gamepad.dpad_up){
            armToggleButton = false;
        }
        return gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                : gamepad.dpad_up ? RobotState.MIN
                :gamepad.x ? RobotState.LOW
                : gamepad.y ? RobotState.MID
                : gamepad.dpad_left ? RobotState.STACK
                : lastState;
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
            case MIN:
                break;

        }
        return stateFromDriver;
    }

    public static void setSubsystemToState(Gamepad gamepad1, Gamepad gamepad2) {
        wanted = getState(gamepad1);

        if (wanted == RobotState.TRAVEL && lastState != RobotState.TRAVEL){
            armDelay.startAction(GlobalData.currentTime);
        }


        switch (wanted) {
            case TRAVEL:
              if (!armToggleButton && armDelay.isDelayPassed()){
                  armState = ArmStates.TRAVEL;
              }
              pinchState = PinchStates.CLOSED;
                break;
            case INTAKE:
                if (!armToggleButton) {
                    armState = ArmStates.GROUND;
                }
                if (gamepad1.left_bumper){
                    pinchState = PinchStates.INTAKELEFT;
                } else if (gamepad1.right_bumper) {
                    pinchState = PinchStates.INTAKERIGHT;
                }else if (gamepad1.a){
                    pinchState = PinchStates.OPEN;
                }
                break;
            case MIN:
                if (!armToggleButton) {
                    armState = ArmStates.MIN;
                }
                if (gamepad1.left_bumper){
                    pinchState = PinchStates.RIGHT;
                } else if (gamepad1.right_bumper) {
                    pinchState = PinchStates.LEFT;
                }

                break;
            case LOW:
                if (!armToggleButton) {
                       armState = ArmStates.LOW;
                }
                if (gamepad1.right_bumper){
                    pinchState = PinchStates.LEFT;
                } else if (gamepad1.left_bumper) {
                    pinchState = PinchStates.RIGHT;
                }
                break;
            case MID:
                if (!armToggleButton) {
                    armState = ArmStates.MID;
                }
                if (gamepad1.left_bumper){
                    pinchState = PinchStates.RIGHT;
                } else if (gamepad1.right_bumper) {
                    pinchState = PinchStates.LEFT;
                }
                break;
            case STACK:
                if (!armToggleButton) {
                    armState = ArmStates.STACK;
                }
                if (gamepad1.left_bumper){
                    pinchState = PinchStates.INTAKELEFT;
                } else if (gamepad1.right_bumper) {
                    pinchState = PinchStates.INTAKERIGHT;
                }else if (gamepad1.a){
                    pinchState = PinchStates.OPEN;
                }
                break;
        }
        if (gamepad1.right_stick_y != 0) {
            armState = ArmStates.OVERRIDE;
            armToggleButton = true;
        }
        Arm.operate(armState, gamepad1, gamepad2);
        Pinch.operate(pinchState);


        lastState = wanted;
        if (gamepad1.back) Plane.operate(PlaneStates.THROW);
        if (gamepad1.dpad_down) OrbitGyro.resetGyro();
    }

    public static void printStates(Telemetry telemetry) {
        telemetry.addData("Robot current state ", SubSystemManager.wanted);
        telemetry.addData("Robot last state", SubSystemManager.lastState);
        telemetry.addData("armPose", Arm.currentPos);
        telemetry.addData("gyro", Math.toDegrees(PoseStorage.currentPose.getHeading()));
        telemetry.addData("lastAngle", OrbitGyro.lastAngle);
        telemetry.addData("currentTime", GlobalData.currentTime);
        telemetry.addData("lastTime", GlobalData.lastTime);
        telemetry.addData("deltaTime",GlobalData.deltaTime);
        telemetry.addData("lf power" , motors[0].getPower());
        telemetry.addData("rf power" , motors[1].getPower());
        telemetry.addData("lb power" , motors[2].getPower());
        telemetry.addData("rb power" , motors[3].getPower());
        telemetry.addData("arm power" , GlobalData.armPower);
//        telemetry.addData("distance in inch", OrbitDistanceSensor.getDistance());
//        telemetry.addData("color", OrbitColorSensor.hasGamePiece());
//        telemetry.addData("magnetic press?", MagneticSensor.getState());
//        telemetry.addData("touchSensor press?", TouchSensor.getState());
//        telemetry.addData("potentiometer", Potentiometer.getVolt());
        telemetry.update();

    }
}

//dani yalechan!
// yoel yalechan!