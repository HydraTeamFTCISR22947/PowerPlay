package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Vector;

public class BasicAutoTester extends LinearOpMode
{
    //start values.
    public static final double startPoseX = -36, startPoseY = -72, startPoseAngle = 90;

    /* NOT FINAL*/
    /*Still need to write actual coordinates*/
    public static final double highBarX = 0, highBarY = 0, highBarAngle = 180;
    public static final double coneStpackX = 0, coneStackY = 0, coneStackAngle = 180;
    public static final double  parkX = 0, parkY = 0, parkAngle = 180;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(startPoseX,startPoseY, Math.toRadians(startPoseAngle));
        Pose2d firstCycleBarPose = new Pose2d(highBarX,highBarY, Math.toRadians(startPoseAngle));
        Vector2d secondCycleBarPose = new Vector2d(highBarX,highBarY);
        Vector2d coneStackPose = new Vector2d(coneStackY,coneStackY);
        Pose2d parkPose = new Pose2d(parkX,parkY, Math.toRadians(parkAngle));


        Trajectory firstCycleScore = drivetrain.trajectoryBuilder(startPose)
                .splineToLinearHeading(firstCycleBarPose,highBarAngle)
                .build();

        Trajectory firstCycleCatch = drivetrain.trajectoryBuilder(firstCycleScore.end())
                .lineTo(coneStackPose)
                .build();

        Trajectory secondCycleScore = drivetrain.trajectoryBuilder(firstCycleCatch.end())
                .lineTo(secondCycleBarPose)
                .build();

        Trajectory park = drivetrain.trajectoryBuilder(secondCycleScore.end())
                .splineToLinearHeading(parkPose,parkAngle)
                .build();

        waitForStart();
        while(opModeIsActive())
        {

            drivetrain.followTrajectory(firstCycleScore);
            drivetrain.followTrajectory(firstCycleCatch);
            drivetrain.followTrajectory(secondCycleScore);
            drivetrain.followTrajectory(park);
        }
    }

}
