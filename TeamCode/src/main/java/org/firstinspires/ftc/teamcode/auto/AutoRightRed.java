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
    public static double startConeDeliveryX = 36 , startConeDeliveryY = -18, startConeDeliveryAngle = 50;
    public static double coneIntakeX = 56 , coneIntakeY = -18, coneIntakeAngle = 0;
    public static double startConeHelpX = 15, secondConeHelpX = 13 , intakeHelpY = 11;
    public static double nearParkX = 36 , nearParkY = -40, parkAngle = 90;
    public static double DELIVERY_WAIT_TIME = 1, RELEASE_WAIT_TIME = 1;

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
        Vector2d startConeDelivery = new Vector2d(startConeDeliveryX, startConeDeliveryY);
        Pose2d intakePose = new Pose2d(coneIntakeX, coneIntakeY,  Math.toRadians(coneIntakeAngle));
        Pose2d nearParkPose = new Pose2d(nearParkX, nearParkY,  Math.toRadians(parkAngle));


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

        MarkerCallback intakeCone =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                clawServo.closeClaw();
            }
        };

        MarkerCallback releaseCone2 =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                transferSystem.pickUp();
                elevatorSystem.baseLevel();
                rotationServo.pickUpPos();
            }
        };

        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drivetrain.trajectorySequenceBuilder(startPose)
                //First cone
                .lineTo(startConeDelivery)
                .turn(Math.toRadians(startConeDeliveryAngle))

                .forward(startConeHelpX)
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(releaseCone1)


                //Second cone
                .back(intakeHelpY)
                .lineToLinearHeading(intakePose)
                .addTemporalMarker(releaseCone2)
                .waitSeconds(RELEASE_WAIT_TIME)

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


