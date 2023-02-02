package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.commands.AutoCatchCommand;
import org.firstinspires.ftc.teamcode.commands.AutoReleaseCommand;
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

    AutoCatchCommand catchCommand;
    AutoReleaseCommand releaseCommand;
    TransferSystem transferSystem;
    RotationServo rotationServo;
    ClawServo clawServo;
    ElevatorSystem elevatorSystem;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(startPosX, startPosY, Math.toRadians(startPosAngle));

        transferSystem = new TransferSystem(hardwareMap);
        rotationServo = new RotationServo(hardwareMap);
        clawServo = new ClawServo(hardwareMap);
        elevatorSystem = new ElevatorSystem(hardwareMap);
        catchCommand = new AutoCatchCommand(hardwareMap);
        releaseCommand = new AutoReleaseCommand(hardwareMap);

        catchCommand.setStackTarget(AutoCatchCommand.StackTarget.FIRST);

        MarkerCallback releaseCone1 =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                clawServo.openClaw();
            }
        };


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

        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(60);

        TrajectorySequence traj1 = drivetrain.trajectorySequenceBuilder(startPose)
                //First cone
                .strafeRight(startConeStrafe1, velConstraint, accelConstraint)
                .strafeLeft(startConeStrafe2)
                .forward(startConeForward)
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(releaseCone1)
                .waitSeconds(RELEASE_WAIT_TIME)
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
                .addTemporalMarker(releaseCone1)
                .waitSeconds(RELEASE_WAIT_TIME)
                .build();

        if (isStopRequested()) {return;}

        clawServo.closeClaw();

        waitForStart();

        elevatorSystem.goToPos(elevatorSystem.BASE_HEIGHT);
        elevatorSystem.midRod();
        rotationServo.pickUpPos();
        transferSystem.highOppositePos();

        drivetrain.followTrajectorySequence(traj1);
    }
}


