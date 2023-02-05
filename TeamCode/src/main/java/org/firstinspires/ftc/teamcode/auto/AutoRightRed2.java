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

import org.firstinspires.ftc.teamcode.commands.auto.AutoCatch;
import org.firstinspires.ftc.teamcode.commands.auto.AutoRelease;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.RotationServo;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;

import java.util.Arrays;


@Config
@Autonomous(name = "Auto Right Red Test 2", group = "auto")
public class AutoRightRed2 extends LinearOpMode {

    public static double startPosX = 36, startPosY = -66, startPosAngle = 180;
    public static double startConeStrafe1 = 58.8, startConeStrafe2 = 19.25, startConeForward = 4.75;
    public static double intakePose1X = 35, intakePose1Y = -20, intakePose1Angle = 180;
    public static double intakePose2X = 58, intakePose2Y = -14.5, intakePose2Angle = 180;
    public static double posConeX = 32, posConeY = -27, posConeAngle = 185;
    public static double DELIVERY_WAIT_TIME = .25, RELEASE_WAIT_TIME = .33;
    public static double PARK_ASSIST = 23, TARGET_ZONE = 24;

    public static int posConeHelperX1 = 1, posConeHelperX2 = 3;

    AutoRelease release;
    AutoCatch autoCatch;
    ClawServo clawServo;
    ElevatorSystem elevatorSystem;
    TransferSystem transferSystem;
    RotationServo rotationServo;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(startPosX, startPosY, Math.toRadians(startPosAngle));
        release = new AutoRelease(hardwareMap);
        autoCatch = new AutoCatch(hardwareMap);
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
                //.waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(release.releaseCone())
                .waitSeconds(RELEASE_WAIT_TIME)
                .build();

        TrajectorySequence cycle1 = drivetrain.trajectorySequenceBuilder(preload.end())
                .lineTo(new Vector2d(intakePose1X, intakePose1Y))
                .addTemporalMarker(release.readyToRelease())
                .splineToLinearHeading(new Pose2d(intakePose2X, intakePose2Y, Math.toRadians(intakePose2Angle)), Math.toRadians(0))
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCatch.intakeFirstCone())
                .addTemporalMarker(autoCatch.catchCone())
                .waitSeconds(DELIVERY_WAIT_TIME*2)
                .build();

        TrajectorySequence place1 = drivetrain.trajectorySequenceBuilder(cycle1.end())
                .setReversed(true)
                .lineTo(new Vector2d(intakePose1X, intakePose1Y))
                .addTemporalMarker(autoCatch.readyToRelease())
                .splineToLinearHeading(new Pose2d(posConeX, posConeY, Math.toRadians(posConeAngle)), Math.toRadians(0))
                .waitSeconds(DELIVERY_WAIT_TIME * 2)
                .addTemporalMarker(release.releaseCone())
                .waitSeconds(DELIVERY_WAIT_TIME)
                .build();


        TrajectorySequence cycle2 = drivetrain.trajectorySequenceBuilder(preload.end())
                .lineTo(new Vector2d(intakePose1X, intakePose1Y))
                .addTemporalMarker(release.readyToRelease())
                .splineToLinearHeading(new Pose2d(intakePose2X, intakePose2Y, Math.toRadians(intakePose2Angle)), Math.toRadians(0))
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCatch.intakeSecondCone())
                .addTemporalMarker(autoCatch.catchCone())
                .waitSeconds(DELIVERY_WAIT_TIME*2)
                .build();

        TrajectorySequence place2 = drivetrain.trajectorySequenceBuilder(cycle2.end())
                .setReversed(true)
                .lineTo(new Vector2d(intakePose1X, intakePose1Y))
                .addTemporalMarker(autoCatch.readyToRelease())
                .splineToLinearHeading(new Pose2d(posConeX + posConeHelperX1, posConeY, Math.toRadians(posConeAngle)), Math.toRadians(0))
                .waitSeconds(DELIVERY_WAIT_TIME * 2)
                .addTemporalMarker(release.releaseCone())
                .waitSeconds(DELIVERY_WAIT_TIME)
                .build();

        TrajectorySequence cycle3 = drivetrain.trajectorySequenceBuilder(preload.end())
                .lineTo(new Vector2d(intakePose1X, intakePose1Y))
                .addTemporalMarker(release.readyToRelease())
                .splineToLinearHeading(new Pose2d(intakePose2X + posConeHelperX2, intakePose2Y, Math.toRadians(intakePose2Angle)), Math.toRadians(0))
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCatch.intakeThirdCone())
                .addTemporalMarker(autoCatch.catchCone())
                .waitSeconds(DELIVERY_WAIT_TIME*2)
                .build();

        TrajectorySequence place3 = drivetrain.trajectorySequenceBuilder(cycle2.end())
                .setReversed(true)
                .lineTo(new Vector2d(intakePose1X, intakePose1Y))
                .addTemporalMarker(autoCatch.readyToRelease())
                .splineToLinearHeading(new Pose2d(posConeX, posConeY, Math.toRadians(posConeAngle)), Math.toRadians(0))
                .waitSeconds(DELIVERY_WAIT_TIME * 2)
                .addTemporalMarker(release.releaseCone())
                .waitSeconds(DELIVERY_WAIT_TIME)
                .build();



        TrajectorySequence park1 = drivetrain.trajectorySequenceBuilder(place1.end())
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .strafeLeft(PARK_ASSIST)
                .back(TARGET_ZONE)
                .build();

        TrajectorySequence park2 = drivetrain.trajectorySequenceBuilder(place1.end())
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .strafeLeft(PARK_ASSIST)
                .build();

        TrajectorySequence park3 = drivetrain.trajectorySequenceBuilder(place1.end())
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .strafeLeft(PARK_ASSIST)
                .forward(TARGET_ZONE)
                .addTemporalMarker(autoCatch.readyToRelease())
                .build();

        if (isStopRequested()) {return;}

        clawServo.closeClaw();

        waitForStart();

        elevatorSystem.goToPos(elevatorSystem.BASE_HEIGHT);
        elevatorSystem.midRod();
        rotationServo.pickUpPos();
        transferSystem.highOppositePos();

        drivetrain.followTrajectorySequence(preload);

        drivetrain.followTrajectorySequence(cycle1);
        drivetrain.followTrajectorySequence(place1);

        drivetrain.followTrajectorySequence(cycle2);
        drivetrain.followTrajectorySequence(place2);

        drivetrain.followTrajectorySequence(cycle3);
        drivetrain.followTrajectorySequence(place3);

        drivetrain.followTrajectorySequence(park2);
    }
}


