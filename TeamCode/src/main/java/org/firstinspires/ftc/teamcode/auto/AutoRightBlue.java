//package org.firstinspires.ftc.teamcode.auto;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
//import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
//
//@Config
//@Autonomous(name = "Auto Right Blue", group = "auto")
//public class AutoRightBlue extends LinearOpMode {
//
//    //start values.
//    public static final double START_POSE_X = -36, START_POSE_Y = 72, START_POSE_ANGLE = 270;
//
//    /* NOT FINAL */
//    public static final double START_CONE_DELIVERY_X = -36, START_CONE_DELIVERY_Y = 0, START_CONE_DELIVERY_ANGLE = 0;
//    public static final double CONE_DELIVERY_POSE_X = -32, CONE_DELIVERY_POSE_Y = 12, CONE_DELIVERY_POSE_ANGLE = 307;
//    public static final double CONE_INTAKE_POSE_X = -56, CONE_INTAKE_POSE_Y = 12, CONE_INTAKE_POSE_ANGLE = 0;
//    //Need to put actual coordinates
//    public static final double  PARK_POSE_X = 0, PARK_POSE_Y = 0, PARK_POSE_ANGLE = 180;
//
//    public static final double TRACK_WIDTH = 13.23;
//    public static final double MAX_VELOCITY = 80;
//    public static final double MAX_ACCELERATION = 60;
//    public static final double MAX_ANGLE_VELOCITY = 180;
//    public static final double MAX_ANGLE_ACCELERATION = 180;
//
//    public static final double DELIVERY_WAIT_TIME = 2, INTAKE_WAIT_TIME = 4;
//    public static final double INTAKE_OFFSET = 1;// move closer to cone stack each cycle ( check this irl )
//
//    ElevatorSystem elevator;
//    TransferSystem transferSystem;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
//
//        elevator = new ElevatorSystem(hardwareMap);
//        transferSystem = new TransferSystem(hardwareMap);
//
//        MarkerCallback elevatorMax = new MarkerCallback() {
//            @Override
//            public void onMarkerReached() {
//
//                elevator.highRod();
//                transferSystem.;
//
//            }
//        };
//
//        Pose2d startPose = new Pose2d(START_POSE_X, START_POSE_Y, Math.toRadians(START_POSE_ANGLE));
//        drivetrain.setPoseEstimate(startPose);
//
//        Pose2d firstCycleBarPose = new Pose2d(START_CONE_DELIVERY_X, START_CONE_DELIVERY_Y, Math.toRadians(START_CONE_DELIVERY_ANGLE));
//        Pose2d secondCycleBarPose = new Pose2d(CONE_DELIVERY_POSE_X, CONE_DELIVERY_POSE_Y, Math.toRadians(CONE_DELIVERY_POSE_ANGLE));
//        Pose2d coneStackPose = new Pose2d(CONE_INTAKE_POSE_X, CONE_INTAKE_POSE_Y, Math.toRadians(CONE_INTAKE_POSE_ANGLE));
//        Pose2d coneStackPoseSecond = new Pose2d(CONE_INTAKE_POSE_X-INTAKE_OFFSET,CONE_INTAKE_POSE_Y,Math.toRadians(CONE_INTAKE_POSE_ANGLE));
//        Pose2d coneStackPoseThird = new Pose2d(CONE_INTAKE_POSE_X-2*INTAKE_OFFSET,CONE_INTAKE_POSE_Y,Math.toRadians(CONE_INTAKE_POSE_ANGLE));
//
//
//        TrajectorySequence autoCycles = drivetrain.trajectorySequenceBuilder(startPose)
//
//                /* First Cycle*/
//                .lineToLinearHeading(firstCycleBarPose)//delivery
//                .waitSeconds(DELIVERY_WAIT_TIME)
//
//                /*Second Cycle*/
//                .splineToLinearHeading(coneStackPose,Math.toRadians(START_POSE_ANGLE))//intake
//                .waitSeconds(INTAKE_WAIT_TIME)
//
//                .lineToLinearHeading(secondCycleBarPose)//delivery
//                .waitSeconds(DELIVERY_WAIT_TIME)
//
//                /*Third Cycle*/
//                .lineToLinearHeading(coneStackPoseSecond)//intake
//                .waitSeconds(INTAKE_WAIT_TIME)
//
//                .lineToLinearHeading(secondCycleBarPose)//delivery
//                .waitSeconds(DELIVERY_WAIT_TIME)
//
//                /*Fourth Cycle*/
//                .lineToLinearHeading(coneStackPoseThird)//intake
//                .waitSeconds(INTAKE_WAIT_TIME)
//
//                .lineToLinearHeading(secondCycleBarPose)//delivery
//                .waitSeconds(DELIVERY_WAIT_TIME)
//
//
//                .build();
//
//        if (isStopRequested())
//        {
//            return;
//        }
//        waitForStart();
//        while(opModeIsActive())
//        {
//            drivetrain.followTrajectorySequence(autoCycles);
//        }
//
//    }
//
//
//}