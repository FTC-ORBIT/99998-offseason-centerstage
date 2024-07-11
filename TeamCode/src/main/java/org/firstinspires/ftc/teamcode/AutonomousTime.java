package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.pinch.Pinch;
import org.firstinspires.ftc.teamcode.robotSubSystems.pinch.PinchStates;

public class AutonomousTime extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pinch.init(hardwareMap,"pinchServo","pinchServo2");
        Drivetrain.init(hardwareMap);
        waitForStart();
        final double startTime = time;
        while (startTime + 0.5 > time){
            Drivetrain.moveFor();
        }
        Drivetrain.stop();
        Pinch.operate(PinchStates.RIGHT);
        final double endTime = time;
        while (endTime + 0.5 > time){
            Drivetrain.moveBack();
        }
        Drivetrain.stop();
    }
}
