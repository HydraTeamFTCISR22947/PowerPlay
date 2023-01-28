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
    public static final double startConeDeliveryPoseX = -36, startConeDeliveryPoseY = 0, startConeDeliveryAngle = 0;
    public static final double coneDeliveryPoseX = -32, coneDeliveryPoseY = -12, coneDeliveryAngle = 30;
    public static final double coneIntakePoseX = -56, coneIntakePoseY = -12, coneIntakeAngle = 0;
    //Need to put actual coordiantes
    public static final double  parkX = 0, parkY = 0, parkAngle = 180;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(startPoseX,startPoseY, Math.toRadians(startPoseAngle));
        drivetrain.setPoseEstimate(startPose);

        Pose2d firstCycleBarPose = new Pose2d(startConeDeliveryPoseX,startConeDeliveryPoseY, Math.toRadians(startConeDeliveryAngle));
        Pose2d secondCycleBarPose = new Pose2d(coneDeliveryPoseX,coneDeliveryPoseY,Math.toRadians(coneDeliveryAngle));
        Pose2d coneStackPose = new Pose2d(coneIntakePoseX,coneIntakePoseY, Math.toRadians(coneIntakeAngle));

        //  Pose2d parkPose = new Pose2d(parkX,parkY, Math.toRadians(parkAngle));


        Trajectory firstCycleDelivery = drivetrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(firstCycleBarPose)
                .build();

        Trajectory firstCycleIntake = drivetrain.trajectoryBuilder(firstCycleDelivery.end())
                .splineToLinearHeading(coneStackPose,Math.toRadians(startPoseAngle))
                .build();

        Trajectory secondCycleDelivery = drivetrain.trajectoryBuilder(firstCycleIntake.end())
                .lineToLinearHeading(secondCycleBarPose)
                .build();

        Trajectory secondCycleIntake = drivetrain.trajectoryBuilder(secondCycleDelivery.end())
                .lineToLinearHeading(coneStackPose)
                .build();

        Trajectory thridCycleDelivery = drivetrain.trajectoryBuilder(secondCycleIntake.end())
                .lineToLinearHeading(secondCycleBarPose)
                .build();

/*
            Trajectory park = drivetrain.trajectoryBuilder(secondCycleScore.end())
                    .splineToLinearHeading(parkPose,parkAngle)
                    .build();
*/
        waitForStart();
        while(opModeIsActive())
        {

            drivetrain.followTrajectory(firstCycleDelivery);
            drivetrain.followTrajectory(firstCycleIntake);

            drivetrain.followTrajectory(secondCycleDelivery);
            drivetrain.followTrajectory(secondCycleIntake);

            drivetrain.followTrajectory(thridCycleDelivery);
            //   drivetrain.followTrajectory(park);
        }
    }

}
