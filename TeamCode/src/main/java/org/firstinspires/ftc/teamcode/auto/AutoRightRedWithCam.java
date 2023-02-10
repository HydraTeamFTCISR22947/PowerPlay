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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.RotationServo;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;


@Config
@Disabled
@Autonomous(name = "Auto Right Red CAM", group = "auto")
public class AutoRightRedWithCam extends LinearOpMode {

    public static double startPosX = 36, startPosY = -66, startPosAngle = 180;
    public static double startConeStrafe1 = 58.8, startConeStrafe2 = 19.5, startConeForward = 4.8;
    public static double intakePose1X = 35, intakePose1Y = -20.3, intakePose1Angle = 180;
    public static double intakePose2X = 62.5, intakePose2Y = -15, intakePose2Angle = 180;
    public static double posCone1X = 43.5, posCone1Y = -18;
    public static double posCone2X = 35.2, posCone2Y = -29.8, posCone2Angle = 180;
    public static double posCone2HelpX = 2, posCone3HelpX = 3;
    public static double DELIVERY_WAIT_TIME = .25, RELEASE_WAIT_TIME = .33, INTAKE_WAIT_TIME = .8, ELEVATOR_WAIT_TIME = .25;
    public static double PARK_ASSIST = 15, TARGET_ZONE = 24;

    AutoCommands autoCommands;
    ClawServo clawServo;
    ElevatorSystem elevatorSystem;
    TransferSystem transferSystem;
    RotationServo rotationServo;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST1 = 0; // Tag ID 0 from the 36h11 family
    int ID_TAG_OF_INTEREST2 = 1; // Tag ID 1 from the 36h11 family
    int ID_TAG_OF_INTEREST3 = 2; // Tag ID 2 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

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
                .setReversed(true)
                .lineTo(new Vector2d(posCone1X, posCone1Y))
                .addTemporalMarker(autoCommands.readyToRelease())
                //.splineToLinearHeading(new Pose2d(posConeX, posConeY, Math.toRadians(posConeAngle)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(posCone2X + posCone2HelpX, posCone2Y, Math.toRadians(posCone2Angle)))
                .waitSeconds(DELIVERY_WAIT_TIME * 2)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(DELIVERY_WAIT_TIME)
                .build();


//        TrajectorySequence cycle2 = drivetrain.trajectorySequenceBuilder(preload.end())
//                .lineTo(new Vector2d(intakePose1X, intakePose1Y))
//                .addTemporalMarker(autoCommands.intakeSecondCone())
//                .splineToLinearHeading(new Pose2d(intakePose2X + 2, intakePose2Y, Math.toRadians(intakePose2Angle)), Math.toRadians(0))
//                .waitSeconds(INTAKE_WAIT_TIME)
//                .addTemporalMarker(autoCommands.catchCone())
//                .waitSeconds(ELEVATOR_WAIT_TIME)
//                .addTemporalMarker(autoCommands.elevatorIntake())
//                .waitSeconds(DELIVERY_WAIT_TIME*2)
//                .build();
//
//        TrajectorySequence place2 = drivetrain.trajectorySequenceBuilder(cycle2.end())
//                .setReversed(true)
//                .lineTo(new Vector2d(posCone1X, posCone1Y))
//                .addTemporalMarker(autoCommands.readyToRelease())
//                //.splineToLinearHeading(new Pose2d(posConeX, posConeY, Math.toRadians(posConeAngle)), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(posCone2X + posCone3HelpX, posCone2Y, Math.toRadians(posCone2Angle)))
//                .waitSeconds(DELIVERY_WAIT_TIME * 2)
//                .addTemporalMarker(autoCommands.releaseCone())
//                .waitSeconds(DELIVERY_WAIT_TIME)
//                .build();
//
//        TrajectorySequence cycle3 = drivetrain.trajectorySequenceBuilder(preload.end())
//                .lineTo(new Vector2d(intakePose1X, intakePose1Y))
//                .addTemporalMarker(autoCommands.intakeThirdCone())
//                .splineToLinearHeading(new Pose2d(intakePose2X, intakePose2Y, Math.toRadians(intakePose2Angle)), Math.toRadians(0))
//                .waitSeconds(INTAKE_WAIT_TIME)
//                .addTemporalMarker(autoCommands.catchCone())
//                .waitSeconds(ELEVATOR_WAIT_TIME)
//                .addTemporalMarker(autoCommands.elevatorIntake())
//                .waitSeconds(DELIVERY_WAIT_TIME * 3)
//                .build();
//
//        TrajectorySequence place3 = drivetrain.trajectorySequenceBuilder(cycle2.end())
//                .setReversed(true)
//                .lineTo(new Vector2d(posCone1X, posCone1Y))
//                .addTemporalMarker(autoCommands.readyToRelease())
//                //.splineToLinearHeading(new Pose2d(posConeX, posConeY, Math.toRadians(posConeAngle)), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(posCone2X, posCone2Y, Math.toRadians(posCone2Angle)))
//                .waitSeconds(DELIVERY_WAIT_TIME * 2)
//                .addTemporalMarker(autoCommands.releaseCone())
//                .waitSeconds(DELIVERY_WAIT_TIME)
//                .build();


        TrajectorySequence park1 = drivetrain.trajectorySequenceBuilder(place1.end())
                .back(5)
                .addTemporalMarker(autoCommands.readyToRelease())
                .strafeLeft(PARK_ASSIST)
                .back(TARGET_ZONE)
                .build();

        TrajectorySequence park2 = drivetrain.trajectorySequenceBuilder(place1.end())
                .back(5)
                .strafeLeft(PARK_ASSIST)
                .addTemporalMarker(autoCommands.readyToRelease())
                .build();

        TrajectorySequence park3 = drivetrain.trajectorySequenceBuilder(place1.end())
                .back(5)
                .addTemporalMarker(autoCommands.readyToRelease())
                .strafeLeft(PARK_ASSIST)
                .forward(TARGET_ZONE)
                .build();

        if (isStopRequested()) {return;}

        clawServo.closeClaw();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST1)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    else if(tag.id == ID_TAG_OF_INTEREST2)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    else if(tag.id == ID_TAG_OF_INTEREST3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        waitForStart();

        elevatorSystem.goToPos(elevatorSystem.BASE_HEIGHT);
        elevatorSystem.midRod();
        rotationServo.pickUpPos();
        transferSystem.highPos();

        drivetrain.followTrajectorySequence(preload);

        drivetrain.followTrajectorySequence(cycle1);
        drivetrain.followTrajectorySequence(place1);

//        drivetrain.followTrajectorySequence(cycle2);
//        drivetrain.followTrajectorySequence(place2);
//
//        drivetrain.followTrajectorySequence(cycle3);
//        drivetrain.followTrajectorySequence(place3);

        switch (tagOfInterest.id)
        {
            case 0:
                drivetrain.followTrajectorySequence(park2);
                break;
            case 1:
                drivetrain.followTrajectorySequence(park1);
                break;
            case 2:
            default:
                drivetrain.followTrajectorySequence(park3);
                break;
        }

        elevatorSystem.baseLevel();
        transferSystem.pickUp();

        while (opModeIsActive());
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}


