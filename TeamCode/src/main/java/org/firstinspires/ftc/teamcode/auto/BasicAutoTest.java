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
    public static final double START_POSE_X = -36, START_POSE_Y = -72, START_POSE_ANGLE = 90;

    /* NOT FINAL*/
    public static final double START_CONE_DELIVERY_X = -36, START_CONE_DELIVERY_Y = 0, START_CONE_DELIVERY_ANGLE = 0;
    public static final double CONE_DELIVERY_POSE_X = -32, CONE_DELIVERY_POSE_Y = -12, CONE_DELIVERY_POSE_ANGLE = 30;
    public static final double CONE_INTAKE_POSE_X = -56, CONE_INTAKE_POSE_Y = -12, CONE_INTAKE_POSE_ANGLE = 0;
    //Need to put actual coordiantes
    public static final double  PARK_POSE_X = 0, PARK_POSE_Y = 0, PARK_POSE_ANGLE = 180;

    public static final double TRACK_WIDTH = 13.23;
    public static final double MAX_VELOCITY = 80;
    public static final double MAX_ACCELERATION = 60;
    public static final double MAX_ANGLE_VELOCITY = 180;
    public static final double MAX_ANGLE_ACCELARATION = 180;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(START_POSE_X, START_POSE_Y, Math.toRadians(START_POSE_ANGLE));
        drivetrain.setPoseEstimate(startPose);

        Pose2d firstCycleBarPose = new Pose2d(START_CONE_DELIVERY_X, START_CONE_DELIVERY_Y, Math.toRadians(START_CONE_DELIVERY_ANGLE));
        Pose2d secondCycleBarPose = new Pose2d(CONE_DELIVERY_POSE_X, CONE_DELIVERY_POSE_Y, Math.toRadians(CONE_DELIVERY_POSE_ANGLE));
        Pose2d coneStackPose = new Pose2d(CONE_INTAKE_POSE_X, CONE_INTAKE_POSE_Y, Math.toRadians(CONE_INTAKE_POSE_ANGLE));

        //  Pose2d parkPose = new Pose2d(parkX,parkY, Math.toRadians(parkAngle));


        Trajectory firstCycleDelivery = drivetrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(firstCycleBarPose)
                .build();

        Trajectory firstCycleIntake = drivetrain.trajectoryBuilder(firstCycleDelivery.end())
                .splineToLinearHeading(coneStackPose,Math.toRadians(START_POSE_ANGLE))
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
