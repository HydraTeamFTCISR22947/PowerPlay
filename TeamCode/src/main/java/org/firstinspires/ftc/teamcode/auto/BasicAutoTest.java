package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Autonomous(name = "Basic Auto Test",group = "auto")
    public class BasicAutoTest extends LinearOpMode
    {
        //start values.
        public static double startPoseX = -36, startPoseY = -72, startPoseAngle = 90;

        /* NOT FINAL*/
        /*Still need to update coordinates*/
        public static final double coneDeliveryPoseX = -36, coneDeliveryPoseY = 0, coneDeliveryAngle = 0;
        public static double coneIntakePoseX = -56, coneIntakePoseY = -12;
        //Need to put actual coordiantes
        public static final double  parkX = 0, parkY = 0, parkAngle = 180;

        @Override
        public void runOpMode() throws InterruptedException {

            SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

            Pose2d startPose = new Pose2d(startPoseX,startPoseY, Math.toRadians(startPoseAngle));
            Pose2d firstCycleBarPose = new Pose2d(coneIntakePoseX,coneIntakePoseY, Math.toRadians(startPoseAngle));
            Vector2d secondCycleBarPose = new Vector2d(coneDeliveryPoseX,coneDeliveryPoseY);
            Vector2d coneStackPose = new Vector2d(coneIntakePoseX,coneIntakePoseY);
          //  Pose2d parkPose = new Pose2d(parkX,parkY, Math.toRadians(parkAngle));


            Trajectory firstCycleScore = drivetrain.trajectoryBuilder(startPose)
                    .splineToLinearHeading(firstCycleBarPose,coneDeliveryAngle)
                    .build();

            Trajectory firstCycleCatch = drivetrain.trajectoryBuilder(firstCycleScore.end())
                    .lineTo(coneStackPose)
                    .build();

            Trajectory secondCycleScore = drivetrain.trajectoryBuilder(firstCycleCatch.end())
                    .lineTo(secondCycleBarPose)
                    .build();
/*
            Trajectory park = drivetrain.trajectoryBuilder(secondCycleScore.end())
                    .splineToLinearHeading(parkPose,parkAngle)
                    .build();
*/
            waitForStart();
            while(opModeIsActive())
            {

                drivetrain.followTrajectory(firstCycleScore);
                drivetrain.followTrajectory(firstCycleCatch);
                drivetrain.followTrajectory(secondCycleScore);
             //   drivetrain.followTrajectory(park);
            }
        }

    }
