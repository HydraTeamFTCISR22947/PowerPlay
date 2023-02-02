package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
//import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;


@Config
@Autonomous(name = "Auto Right Red", group = "auto")
public class AutoRightRed extends LinearOpMode {

 //   ElevatorSystem elevator = new ElevatorSystem(hardwareMap);
  //  TransferSystem transferSystem;
    //start values.
    public static final double START_POSE_X = 36, START_POSE_Y = -72, START_POSE_ANGLE = 90;

    /* NOT FINAL */
    public static double START_CONE_DELIVERY_X = 32, START_CONE_DELIVERY_Y = -24, START_CONE_DELIVERY_ANGLE = 60;
    public static double CONE_DELIVERY_POSE_X = 32, CONE_DELIVERY_POSE_Y = -24, CONE_DELIVERY_POSE_ANGLE = 60;
    public static double CONE_INTAKE_POSE_X = 56, CONE_INTAKE_POSE_Y = -12, CONE_INTAKE_POSE_ANGLE = 180;

    public static double NEAR_CONE_INTAKE_POSE_X = 44, NEAR_CONE_INTAKE_POSE_Y = -14;
    //Need to put actual coordinates

    public static final double DELIVERY_WAIT_TIME = 1, INTAKE_WAIT_TIME = 2;
    public static final double INTAKE_OFFSET = 1;// move closer to cone stack each cycle ( check this irl )


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(START_POSE_X, START_POSE_Y, Math.toRadians(START_POSE_ANGLE));
        drivetrain.setPoseEstimate(startPose);

        Pose2d firstCycleBarPose = new Pose2d(START_CONE_DELIVERY_X, START_CONE_DELIVERY_Y, Math.toRadians(START_POSE_ANGLE));
        Vector2d firstCycleBarVector = new Vector2d(START_CONE_DELIVERY_X, START_CONE_DELIVERY_Y);
        Pose2d secondCycleBarPose = new Pose2d(CONE_DELIVERY_POSE_X, CONE_DELIVERY_POSE_Y, Math.toRadians(CONE_DELIVERY_POSE_ANGLE));

        Pose2d nearConeStackPose = new Pose2d(NEAR_CONE_INTAKE_POSE_X, NEAR_CONE_INTAKE_POSE_Y, Math.toRadians(CONE_INTAKE_POSE_ANGLE));
        Pose2d coneStackPose = new Pose2d(CONE_INTAKE_POSE_X, CONE_INTAKE_POSE_Y, Math.toRadians(CONE_INTAKE_POSE_ANGLE));


        Pose2d coneStackPoseSecond = new Pose2d(CONE_INTAKE_POSE_X + INTAKE_OFFSET, CONE_INTAKE_POSE_Y, Math.toRadians(CONE_INTAKE_POSE_ANGLE));
        Pose2d coneStackPoseThird = new Pose2d(CONE_INTAKE_POSE_X + 2 * INTAKE_OFFSET, CONE_INTAKE_POSE_Y, Math.toRadians(CONE_INTAKE_POSE_ANGLE));


        /*
        MarkerCallback ElevatorMax = new MarkerCallback() {
            @Override
            public void onMarkerReached() {

                elevator.highRod();
            }


        };*/

            /* First Cycle*/
            TrajectorySequence firstCycle =drivetrain.trajectorySequenceBuilder(startPose)

                    .lineToLinearHeading(firstCycleBarPose,SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .turn( Math.toRadians(START_CONE_DELIVERY_ANGLE))
                    .waitSeconds(DELIVERY_WAIT_TIME)
                    .build();

            TrajectorySequence autoCycles = drivetrain.trajectorySequenceBuilder(firstCycle.end())



                    /*Second Cycle*/
                    .splineToLinearHeading(nearConeStackPose,Math.toRadians(CONE_DELIVERY_POSE_ANGLE))
                    .lineToLinearHeading(coneStackPose)
                    .build();



        if (isStopRequested())
        {
                return;
        }
            waitForStart();
        while(opModeIsActive())
        {
            drivetrain.followTrajectorySequence(firstCycle);
            drivetrain.followTrajectorySequence(autoCycles);

        }




    }
}


