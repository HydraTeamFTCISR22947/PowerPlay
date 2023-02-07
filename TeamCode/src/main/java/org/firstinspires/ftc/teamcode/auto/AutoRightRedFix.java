package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.RotationServo;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;

import java.util.Arrays;


@Config
@Autonomous(name = "Auto Right Red FIX", group = "auto")
public class AutoRightRedFix extends LinearOpMode {

    public static double startPosX = 36, startPosY = -66, startPosAngle = 180;
    public static double startConeStrafe1 = 58.8, startConeStrafe2 = 19.5, startConeForward = 4.5;
    public static double intakePose1X = 35, intakePose1Y = -20.3, intakePose1Angle = 180;
    public static double intakePose2X = 62.5, intakePose2Y = -15, intakePose2Angle = 180;
    public static double intakePoseCycleX = 38;
    public static double posCone1X = 40, posCone1Y = -15;
    public static double posCone2X = 28, posCone2Y = -21, posCone2Angle = 225;
    public static double PARK_ASSIST = 20, TARGET_ZONE = 20;

    public static double DELIVERY_WAIT_TIME = .25, RELEASE_WAIT_TIME = .33, INTAKE_WAIT_TIME = .8, ELEVATOR_WAIT_TIME = .25;

    AutoCommands autoCommands;
    ClawServo clawServo;
    ElevatorSystem elevatorSystem;
    TransferSystem transferSystem;
    RotationServo rotationServo;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(startPosX, startPosY, Math.toRadians(startPosAngle));
        autoCommands = new AutoCommands(hardwareMap);
        clawServo = new ClawServo(hardwareMap);
        elevatorSystem = new ElevatorSystem(hardwareMap);
        transferSystem = new TransferSystem(hardwareMap);
        rotationServo = new RotationServo(hardwareMap);


        drivetrain.setPoseEstimate(startPose);

        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(70), new TranslationalVelocityConstraint(70)));

        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL);

        TrajectorySequence preload = drivetrain.trajectorySequenceBuilder(startPose)
                //First cone
                .strafeRight(startConeStrafe1, velConstraint, accelConstraint)
                .strafeLeft(startConeStrafe2)
                .forward(startConeForward)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(RELEASE_WAIT_TIME)
                .build();

        TrajectorySequence cycle1 = drivetrain.trajectorySequenceBuilder(preload.end())
                .lineTo(new Vector2d(intakePose1X, intakePose1Y))
                .addTemporalMarker(autoCommands.intakeFirstCone())
                .splineToLinearHeading(new Pose2d(intakePose2X, intakePose2Y, Math.toRadians(intakePose2Angle)), Math.toRadians(0))
                .waitSeconds(INTAKE_WAIT_TIME)
                .addTemporalMarker(autoCommands.catchCone())
                .waitSeconds(ELEVATOR_WAIT_TIME)
                .addTemporalMarker(autoCommands.elevatorIntake())
                .waitSeconds(DELIVERY_WAIT_TIME*2)
                .build();

        TrajectorySequence place1 = drivetrain.trajectorySequenceBuilder(cycle1.end())
                .lineTo(new Vector2d(posCone1X, posCone1Y))
                .addTemporalMarker(autoCommands.readyToRelease())
                .splineTo(new Vector2d(posCone2X, posCone2Y), Math.toRadians(posCone2Angle))
                .waitSeconds(DELIVERY_WAIT_TIME * 2)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(DELIVERY_WAIT_TIME)
                .build();


        TrajectorySequence cycle2 = drivetrain.trajectorySequenceBuilder(place1.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(intakePoseCycleX, intakePose2Y, Math.toRadians(intakePose1Angle)), Math.toRadians(0))
                .addTemporalMarker(autoCommands.intakeSecondCone())
                .lineTo(new Vector2d(intakePose2X, intakePose2Y))
                .waitSeconds(INTAKE_WAIT_TIME)
                .addTemporalMarker(autoCommands.catchCone())
                .waitSeconds(ELEVATOR_WAIT_TIME)
                .addTemporalMarker(autoCommands.elevatorIntake())
                .waitSeconds(DELIVERY_WAIT_TIME*2)
                .build();

        TrajectorySequence place2 = drivetrain.trajectorySequenceBuilder(cycle2.end())
                .lineTo(new Vector2d(posCone1X, posCone1Y))
                .addTemporalMarker(autoCommands.readyToRelease())
                .splineTo(new Vector2d(posCone2X, posCone2Y), Math.toRadians(posCone2Angle))
                .waitSeconds(DELIVERY_WAIT_TIME * 2)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(DELIVERY_WAIT_TIME)
                .build();

        TrajectorySequence cycle3 = drivetrain.trajectorySequenceBuilder(place2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(intakePoseCycleX, intakePose2Y, Math.toRadians(intakePose1Angle)), Math.toRadians(0))
                .addTemporalMarker(autoCommands.intakeSecondCone())
                .lineTo(new Vector2d(intakePose2X, intakePose2Y))
                .waitSeconds(INTAKE_WAIT_TIME)
                .addTemporalMarker(autoCommands.catchCone())
                .waitSeconds(ELEVATOR_WAIT_TIME)
                .addTemporalMarker(autoCommands.elevatorIntake())
                .waitSeconds(DELIVERY_WAIT_TIME*2)
                .build();

        TrajectorySequence place3 = drivetrain.trajectorySequenceBuilder(cycle3.end())
                .lineTo(new Vector2d(posCone1X, posCone1Y))
                .addTemporalMarker(autoCommands.readyToRelease())
                .splineTo(new Vector2d(posCone2X, posCone2Y), Math.toRadians(posCone2Angle))
                .waitSeconds(DELIVERY_WAIT_TIME * 2)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(DELIVERY_WAIT_TIME)
                .build();


        TrajectorySequence park1 = drivetrain.trajectorySequenceBuilder(place3.end())
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .addTemporalMarker(autoCommands.readyToRelease())
                .strafeLeft(PARK_ASSIST)
                .back(TARGET_ZONE)
                .build();

        TrajectorySequence park2 = drivetrain.trajectorySequenceBuilder(place3.end())
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .strafeLeft(PARK_ASSIST)
                .addTemporalMarker(autoCommands.readyToRelease())
                .build();

        TrajectorySequence park3 = drivetrain.trajectorySequenceBuilder(place3.end())
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .addTemporalMarker(autoCommands.readyToRelease())
                .strafeLeft(PARK_ASSIST)
                .forward(TARGET_ZONE)
                .build();

        if (isStopRequested()) {return;}

        clawServo.closeClaw();

        waitForStart();

        elevatorSystem.goToPos(elevatorSystem.BASE_HEIGHT);
        elevatorSystem.midRod();
        rotationServo.pickUpPos();
        transferSystem.highPos();

        drivetrain.followTrajectorySequence(preload);

        drivetrain.followTrajectorySequence(cycle1);
        drivetrain.followTrajectorySequence(place1);

        drivetrain.followTrajectorySequence(cycle2);
        drivetrain.followTrajectorySequence(place2);

        drivetrain.followTrajectorySequence(cycle2);
        drivetrain.followTrajectorySequence(place2);

        drivetrain.followTrajectorySequence(cycle3);
        drivetrain.followTrajectorySequence(place3);

        drivetrain.followTrajectorySequence(park2);

        elevatorSystem.baseLevel();
        transferSystem.pickUp();

        while (opModeIsActive());
    }
}