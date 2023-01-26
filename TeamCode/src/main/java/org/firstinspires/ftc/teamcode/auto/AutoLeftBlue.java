package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Autonomous(name = "Auto Left Blue", group = "test")
public class AutoLeftBlue extends LinearOpMode {

    double startPoseX = -36, startPoseY = -72, startPoseAngle = 90;


    Pose2d startPose = new Pose2d(startPoseX,startPoseY, Math.toRadians(startPoseAngle));




    @Override
    public void runOpMode() throws InterruptedException {

    }


}