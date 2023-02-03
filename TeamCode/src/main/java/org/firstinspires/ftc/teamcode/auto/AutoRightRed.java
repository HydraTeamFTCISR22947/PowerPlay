package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
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
@Autonomous(name = "Auto Right Red", group = "auto")
public class AutoRightRed extends LinearOpMode {

    public static double startPosX = 36, startPosY = -66, startPosAngle = 180;
    public static double startConeStrafe1 = 58.8, startConeStrafe2 = 19.25, startConeForward = 4.75;
    public static double intakePose1X = 35, intakePose1Y = -13.5, intakePose1Angle = 180;
    public static double intakePose2X = 53, intakePose2Y = -13.5, intakePose2Angle = 180;
    public static double conePosX = 27, posConeY = -20, posConeAngle = -135;
    public static double DELIVERY_WAIT_TIME = .25, RELEASE_WAIT_TIME = .33;
    public static double PARK_ASSIST = 36, TARGET_ZONE = 24;

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

        MarkerCallback releaseCone2 =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                transferSystem.pickUpOpposite();
                elevatorSystem.baseLevel();
                rotationServo.releasePos();
            }
        };

        MarkerCallback getCone =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                rotationServo.releasePos();
            }
        };

        MarkerCallback getCone2 =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                clawServo.closeClaw();
            }
        };

        MarkerCallback getCone3 =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                elevatorSystem.midRod();
            }
        };

        MarkerCallback placeCone =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                transferSystem.highOppositePos();
                elevatorSystem.midRod();
                rotationServo.pickUpPos();
            }
        };

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
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .addTemporalMarker(releaseCone2)
                .lineToLinearHeading(new Pose2d(intakePose2X, intakePose2Y, Math.toRadians(intakePose2Angle)))
                .addTemporalMarker(getCone)
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(getCone2)
                .waitSeconds(DELIVERY_WAIT_TIME*2)
                .addTemporalMarker(getCone3)
                .waitSeconds(DELIVERY_WAIT_TIME)
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .lineToLinearHeading(new Pose2d(conePosX, posConeY, Math.toRadians(posConeAngle)))
                .addTemporalMarker(placeCone)
                .waitSeconds(RELEASE_WAIT_TIME * 2)
                .addTemporalMarker(release.releaseCone())
                .waitSeconds(RELEASE_WAIT_TIME)
                .build();

        TrajectorySequence cycle2 = drivetrain.trajectorySequenceBuilder(cycle1.end())
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .addTemporalMarker(releaseCone2)
                .lineToLinearHeading(new Pose2d(intakePose2X, intakePose2Y, Math.toRadians(intakePose2Angle)))
                .addTemporalMarker(getCone)
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(getCone2)
                .waitSeconds(DELIVERY_WAIT_TIME*2)
                .addTemporalMarker(getCone3)
                .waitSeconds(DELIVERY_WAIT_TIME)
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .lineToLinearHeading(new Pose2d(conePosX, posConeY, Math.toRadians(posConeAngle)))
                .addTemporalMarker(placeCone)
                .waitSeconds(RELEASE_WAIT_TIME * 2)
                .addTemporalMarker(release.releaseCone())
                .waitSeconds(RELEASE_WAIT_TIME)
                .build();

        TrajectorySequence cycle3 = drivetrain.trajectorySequenceBuilder(cycle2.end())
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .addTemporalMarker(releaseCone2)
                .lineToLinearHeading(new Pose2d(intakePose2X, intakePose2Y, Math.toRadians(intakePose2Angle)))
                .addTemporalMarker(getCone)
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(getCone2)
                .waitSeconds(DELIVERY_WAIT_TIME*2)
                .addTemporalMarker(getCone3)
                .waitSeconds(DELIVERY_WAIT_TIME)
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .lineToLinearHeading(new Pose2d(conePosX, posConeY, Math.toRadians(posConeAngle)))
                .addTemporalMarker(placeCone)
                .waitSeconds(RELEASE_WAIT_TIME * 2)
                .addTemporalMarker(release.releaseCone())
                .waitSeconds(RELEASE_WAIT_TIME)
                .build();

        TrajectorySequence park1 = drivetrain.trajectorySequenceBuilder(cycle2.end())
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .strafeRight(PARK_ASSIST)
                .back(TARGET_ZONE)
                .build();

        TrajectorySequence park2 = drivetrain.trajectorySequenceBuilder(cycle2.end())
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .strafeRight(PARK_ASSIST)
                .build();

        TrajectorySequence park3 = drivetrain.trajectorySequenceBuilder(cycle2.end())
                .lineToLinearHeading(new Pose2d(intakePose1X, intakePose1Y, Math.toRadians(intakePose1Angle)))
                .strafeRight(PARK_ASSIST)
                .forward(TARGET_ZONE)
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
        drivetrain.followTrajectorySequence(cycle2);
        drivetrain.followTrajectorySequence(cycle3);
        drivetrain.followTrajectorySequence(park2);
    }
}


