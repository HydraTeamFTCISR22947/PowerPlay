package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Auto Right Red", group = "auto")
public class AutoRightRed extends LinearOpMode {

    //start values.
    public static final double START_POSE_X = 36, START_POSE_Y = -72, START_POSE_ANGLE = 90;

    /* NOT FINAL */
    public static double START_CONE_DELIVERY_X = 36, START_CONE_DELIVERY_Y = -16, START_CONE_DELIVERY_ANGLE = 127;
    public static double CONE_DELIVERY_POSE_X = 36, CONE_DELIVERY_POSE_Y = -12, CONE_DELIVERY_POSE_ANGLE = 127;
    public static double CONE_INTAKE_POSE_X = 56, CONE_INTAKE_POSE_Y = -12, CONE_INTAKE_POSE_ANGLE = 180;
    //Need to put actual coordinates

    public static final double DELIVERY_WAIT_TIME = 1, INTAKE_WAIT_TIME = 2;
    public static final double INTAKE_OFFSET = 1;// move closer to cone stack each cycle ( check this irl )


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(START_POSE_X, START_POSE_Y, Math.toRadians(START_POSE_ANGLE));
        drivetrain.setPoseEstimate(startPose);

        Pose2d coneStackPose1 = new Pose2d(56,-12,Math.toRadians(180));
        Pose2d firstCycleBarPose = new Pose2d(START_CONE_DELIVERY_X, START_CONE_DELIVERY_Y, Math.toRadians(START_CONE_DELIVERY_ANGLE));
        Pose2d secondCycleBarPose = new Pose2d(CONE_DELIVERY_POSE_X, CONE_DELIVERY_POSE_Y, Math.toRadians(CONE_DELIVERY_POSE_ANGLE));
        Pose2d coneStackPose = new Pose2d(CONE_INTAKE_POSE_X, CONE_INTAKE_POSE_Y, Math.toRadians(CONE_INTAKE_POSE_ANGLE));
        Pose2d coneStackPoseSecond = new Pose2d(CONE_INTAKE_POSE_X+INTAKE_OFFSET,CONE_INTAKE_POSE_Y,Math.toRadians(CONE_INTAKE_POSE_ANGLE));
        Pose2d coneStackPoseThird = new Pose2d(CONE_INTAKE_POSE_X+2*INTAKE_OFFSET,CONE_INTAKE_POSE_Y,Math.toRadians(CONE_INTAKE_POSE_ANGLE));



        TrajectorySequence autoCycles = drivetrain.trajectorySequenceBuilder(startPose)

                /* First Cycle*/
                .lineToLinearHeading(firstCycleBarPose)//delivery
                .waitSeconds(DELIVERY_WAIT_TIME)

                /*Second Cycle*/
             //   .splineToLinearHeading(coneStackPose,Math.toRadians(200))//intake

                .lineToSplineHeading(coneStackPose1)
                .splineToLinearHeading(coneStackPose,Math.toRadians(START_POSE_ANGLE))//intake

//                .lineToSplineHeading(coneStackPose)
                .waitSeconds(INTAKE_WAIT_TIME)

                .lineToLinearHeading(secondCycleBarPose)//delivery
                .waitSeconds(DELIVERY_WAIT_TIME)

                /*Third Cycle*/
                .splineToLinearHeading(coneStackPose1,Math.toRadians(START_POSE_ANGLE))//intake
                .lineToSplineHeading(coneStackPose)

//                .lineToLinearHeading(coneStackPoseSecond)//intake
                .waitSeconds(INTAKE_WAIT_TIME)

                .lineToLinearHeading(secondCycleBarPose)//delivery
                .waitSeconds(DELIVERY_WAIT_TIME)

                .build();

        if (isStopRequested())
        {
            return;
        }
        waitForStart();
        while(opModeIsActive())
        {
            drivetrain.followTrajectorySequence(autoCycles);
        }

    }


}