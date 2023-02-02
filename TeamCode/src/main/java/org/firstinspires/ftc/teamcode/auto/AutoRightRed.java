package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
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


@Config
@Autonomous(name = "Auto Right Red", group = "auto")
public class AutoRightRed extends LinearOpMode {

    public static double startPosX = 36, startPosY = -66, startPosAngle = 90;
    public static double startConeX = 36 , startConeY = -18, startConeAngle = 135;
    public static double startConeHelpX = 12;
    public static double nearParkX = 36 , nearParkY = -40, parkAngle = 90;
    public static double DELIVERY_WAIT_TIME = 1.5, INTAKE_WAIT_TIME = 1.0;

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
        Pose2d startCone = new Pose2d(startConeX, startConeY, Math.toRadians(startConeAngle));
        Pose2d nearParkPose = new Pose2d(nearParkX, nearParkY,  Math.toRadians(parkAngle));

        transferSystem = new TransferSystem(hardwareMap);
        rotationServo = new RotationServo(hardwareMap);
        clawServo = new ClawServo(hardwareMap);
        elevatorSystem = new ElevatorSystem(hardwareMap);
        catchCommand = new AutoCatchCommand(hardwareMap);
        releaseCommand = new AutoReleaseCommand(hardwareMap);

        catchCommand.setStackTarget(AutoCatchCommand.StackTarget.FIRST);

        MarkerCallback releaseCone =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                clawServo.openClaw();
                transferSystem.pickUp();
                elevatorSystem.baseLevel();
                rotationServo.pickUpPos();
            }
        };

        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drivetrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(startCone)
                .forward(startConeHelpX)
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(releaseCone)
                .back(startConeHelpX)
                .lineToLinearHeading(nearParkPose)
                .build();

        if (isStopRequested()) {return;}

        clawServo.closeClaw();

        waitForStart();

        elevatorSystem.goToPos(elevatorSystem.BASE_HEIGHT);
        elevatorSystem.highRod();
        rotationServo.pickUpPos();
        transferSystem.highOppositePos();

        drivetrain.followTrajectorySequence(traj1);
    }
}


