package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveByAprilTags.Camera;
import org.firstinspires.ftc.teamcode.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.positionTracker.PoseStorage;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.ArmStates;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrain.Drivetrain;

@Config
@TeleOp(name = "test")
public class Test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();


        ElapsedTime robotTime = new ElapsedTime();
        robotTime.reset();
        Drivetrain.init(hardwareMap);
        OrbitGyro.init(hardwareMap);
        Camera.initAprilTag(hardwareMap,telemetry);


        OrbitGyro.resetGyroStartTeleop((float) Math.toDegrees(PoseStorage.currentPose.getHeading()));
        telemetry.addData("gyro", Math.toDegrees(PoseStorage.currentPose.getHeading()));
        telemetry.addData("lastAngle", OrbitGyro.lastAngle);
        telemetry.update();

        GlobalData.inAutonomous = false;
        GlobalData.currentTime = 0;
        GlobalData.lastTime = 0;
        GlobalData.deltaTime = 0;
        GlobalData.robotState = RobotState.TRAVEL;
        GlobalData.hasGamePiece = false;

        Arm.init(hardwareMap,"armMotor");
        waitForStart();


        while (!isStopRequested()) {
        Arm.operate(ArmStates.OVERRIDE,gamepad1,gamepad2);
        telemetry.addData("armPose", Arm.currectPos);
        telemetry.update();
            }
        }
    }
//dani yalechan!
// yoel yalechan!