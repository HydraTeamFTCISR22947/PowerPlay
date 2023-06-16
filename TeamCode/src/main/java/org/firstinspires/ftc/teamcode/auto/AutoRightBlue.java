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
@Autonomous(name = "Auto Right Blue", group = "auto")
public class AutoRightBlue extends LinearOpMode {
    public static boolean useCamera = false;

    public static double startPosX = 36, startPosY = -65.9, startPosAngle = 180;
    public static double startConeStrafe1 = 54.8, startConeStrafe2 = 15.9, startConeForward = 5.15;


    public static double backIntakeOffset  = 13;


    public static double intakePose1XFirstCone = 36, intakePose1YFirstCone = -19;
    public static double intakePose2XFirstCone = 46.7, intakePose2YFirstCone = -15.6;


    public static double intakePoseCycleXSecondCone = 38, intakePose1YSecondCone = -15.7;
    public static double intakePose2XSecondCone = 50, intakePose2YSecondCone = -15.4;


    public static double intakePoseCycleXThirdCone = 38, intakePose1YThirdCone = -15.7;
    public static double intakePose2XThirdCone = 51, intakePose2YThirdCone = -15;


    public static double intakeAngle = 180;


    public static double posCone1X = 40, posCone1Y = -15, posConeAngle = 225;


    public static double posCone2XFirstCone = 31.5, posCone2YFirstCone = -18.05;


    public static double posCone2XSecondCone = 32, posCone2YSecondCone = -17.4;


    public static double posCone2XThirdCone = 32.5, posCone2YThirdCone = -17.2;


    public static double parkPoseX = 39.3, parkPoseY = -20, parkPoseAngle = 180;


    public static double TARGET_ZONE = 20, GO_TO_PARK_HELPER = 8;


    public static double BACK_WAIT_TIME = 0.1, DELIVERY_WAIT_TIME = .25, RELEASE_WAIT_TIME = .33;
    public static double ALMOST_RELEASE_TIME = 0.1, INTAKE_WAIT_TIME = .1, ELEVATOR_WAIT_TIME = .5;

    final int ID_TAG_OF_INTEREST1 = 0, ID_TAG_OF_INTEREST2 = 1, ID_TAG_OF_INTEREST3 = 2; // Tags from the 36h11 family

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    enum ParkIn
    {
        LEFT,
        CENTER,
        RIGHT
    }

    ParkIn parkIn = ParkIn.CENTER;

    AutoCommands autoCommands;
    ClawServo clawServo;
    ElevatorSystem elevatorSystem;
    TransferSystem transferSystem;
    RotationServo rotationServo;

    @Override
    public void runOpMode() throws InterruptedException {
        if(useCamera)
        {
            initCamera();
        }

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

        TrajectorySequence placePreload = drivetrain.trajectorySequenceBuilder(startPose)
                //Getting to position
                .strafeRight(startConeStrafe1, velConstraint, accelConstraint)
                .strafeLeft(startConeStrafe2, velConstraint, accelConstraint)
                .forward(startConeForward)
                //placing
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCommands.goDownToReleaseCone())
                .waitSeconds(ALMOST_RELEASE_TIME)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(RELEASE_WAIT_TIME)
                .build();
     /*   TrajectorySequence preload = drivetrain.trajectorySequenceBuilder(startPose)
                .strafeRight(startConeStrafe1, velConstraint, accelConstraint)
                .strafeLeft(startConeStrafe2, velConstraint, accelConstraint)
                .forward(startConeForward, velConstraint, accelConstraint)
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCommands.goDownToReleaseCone())
                .waitSeconds(ALMOST_RELEASE_TIME)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(RELEASE_WAIT_TIME)
                .build();

        TrajectorySequence cycle1 = drivetrain.trajectorySequenceBuilder(preload.end())
                .lineTo(new Vector2d(intakePose1XFirstCone, intakePose1YFirstCone))
                .addTemporalMarker(autoCommands.intakeFirstCone())
                .splineToLinearHeading(new Pose2d(intakePose2XFirstCone, intakePose2YFirstCone, Math.toRadians(intakeAngle)), Math.toRadians(0))
                .waitSeconds(BACK_WAIT_TIME)
                .back(backIntakeOffset)
                .waitSeconds(INTAKE_WAIT_TIME)
                .addTemporalMarker(autoCommands.catchCone())
                .waitSeconds(ELEVATOR_WAIT_TIME)
                .addTemporalMarker(autoCommands.elevatorIntake())
                //.waitSeconds(DELIVERY_WAIT_TIME*3)
                .build();

        TrajectorySequence place1 = drivetrain.trajectorySequenceBuilder(cycle1.end())
                .lineTo(new Vector2d(posCone1X, posCone1Y))
                .addTemporalMarker(autoCommands.readyToRelease())
                .splineTo(new Vector2d(posCone2XFirstCone, posCone2YFirstCone), Math.toRadians(posConeAngle))
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCommands.goDownToReleaseCone())
                .waitSeconds(ALMOST_RELEASE_TIME)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(RELEASE_WAIT_TIME)
                .build();


        TrajectorySequence cycle2 = drivetrain.trajectorySequenceBuilder(place1.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(intakePoseCycleXSecondCone, intakePose1YSecondCone, Math.toRadians(intakeAngle)), Math.toRadians(0))
                .addTemporalMarker(autoCommands.intakeSecondCone())
                .lineTo(new Vector2d(intakePose2XSecondCone, intakePose2YSecondCone))
                .waitSeconds(BACK_WAIT_TIME)
                .back(backIntakeOffset)
                .waitSeconds(INTAKE_WAIT_TIME)
                .addTemporalMarker(autoCommands.catchCone())
                .waitSeconds(ELEVATOR_WAIT_TIME)
                .addTemporalMarker(autoCommands.elevatorIntake())
                //.waitSeconds(DELIVERY_WAIT_TIME*2)
                .build();

        TrajectorySequence place2 = drivetrain.trajectorySequenceBuilder(cycle2.end())
                .lineTo(new Vector2d(posCone1X, posCone1Y))
                .addTemporalMarker(autoCommands.readyToRelease())
                .splineTo(new Vector2d(posCone2XSecondCone, posCone2YSecondCone), Math.toRadians(posConeAngle))
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCommands.goDownToReleaseCone())
                .waitSeconds(ALMOST_RELEASE_TIME)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(RELEASE_WAIT_TIME)
                .build();

        TrajectorySequence cycle3 = drivetrain.trajectorySequenceBuilder(place2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(intakePoseCycleXThirdCone, intakePose1YThirdCone, Math.toRadians(intakeAngle)), Math.toRadians(0))
                .addTemporalMarker(autoCommands.intakeThirdCone())
                .lineTo(new Vector2d(intakePose2XThirdCone, intakePose2YThirdCone))
                .waitSeconds(BACK_WAIT_TIME)
                .back(backIntakeOffset)
                .waitSeconds(INTAKE_WAIT_TIME)
                .addTemporalMarker(autoCommands.catchCone())
                .waitSeconds(ELEVATOR_WAIT_TIME)
                .addTemporalMarker(autoCommands.elevatorIntake())
                //.waitSeconds(DELIVERY_WAIT_TIME*2)
                .build();

        TrajectorySequence place3 = drivetrain.trajectorySequenceBuilder(cycle3.end())
                .lineTo(new Vector2d(posCone1X, posCone1Y))
                .addTemporalMarker(autoCommands.readyToRelease())
                .splineTo(new Vector2d(posCone2XThirdCone, posCone2YThirdCone), Math.toRadians(posConeAngle))
                .waitSeconds(DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCommands.goDownToReleaseCone())
                .waitSeconds(ALMOST_RELEASE_TIME)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(RELEASE_WAIT_TIME)
                .build();
*/

        TrajectorySequence park1 = drivetrain.trajectorySequenceBuilder(placePreload.end())
                .back(GO_TO_PARK_HELPER,velConstraint,accelConstraint)
                .addTemporalMarker(autoCommands.readyToRelease())
                .addTemporalMarker(autoCommands.reset())
                .lineToLinearHeading(new Pose2d(parkPoseX, parkPoseY, Math.toRadians(parkPoseAngle)),velConstraint,accelConstraint)
                .back(TARGET_ZONE)
                .build();

        TrajectorySequence park2 = drivetrain.trajectorySequenceBuilder(placePreload.end())
                .back(GO_TO_PARK_HELPER,velConstraint,accelConstraint)
                .addTemporalMarker(autoCommands.readyToRelease())
                .addTemporalMarker(autoCommands.reset())
                .lineToLinearHeading(new Pose2d(parkPoseX, parkPoseY, Math.toRadians(parkPoseAngle)),velConstraint,accelConstraint)
                .build();

        TrajectorySequence park3 = drivetrain.trajectorySequenceBuilder(placePreload.end())
                .back(GO_TO_PARK_HELPER,velConstraint,accelConstraint)
                .addTemporalMarker(autoCommands.readyToRelease())
                .addTemporalMarker(autoCommands.reset())
                .lineToLinearHeading(new Pose2d(parkPoseX, parkPoseY, Math.toRadians(parkPoseAngle)),velConstraint,accelConstraint)
                .forward(TARGET_ZONE)
                .build();

        if (isStopRequested()) {return;}

        clawServo.closeClaw();

        if(useCamera)
        {
            lookWhereToPark();
        }

        waitForStart();

        elevatorSystem.goToPos(elevatorSystem.BASE_HEIGHT);
        elevatorSystem.midRod();
        rotationServo.pickUpPos();
        transferSystem.highPos();

        drivetrain.followTrajectorySequence(placePreload);
/*
        drivetrain.followTrajectorySequence(cycle1);
        drivetrain.followTrajectorySequence(place1);

        drivetrain.followTrajectorySequence(cycle2);
        drivetrain.followTrajectorySequence(place2);

        drivetrain.followTrajectorySequence(cycle3);
        drivetrain.followTrajectorySequence(place3);
*/
        if(useCamera)
        {
            switch (parkIn)
            {
                case LEFT:
                    drivetrain.followTrajectorySequence(park1);
                    break;
                case RIGHT:
                    drivetrain.followTrajectorySequence(park3);
                    break;
                case CENTER:
                default:
                    drivetrain.followTrajectorySequence(park2);
                    break;
            }
        }
        else
        {
            drivetrain.followTrajectorySequence(park2);
        }

        while (opModeIsActive());
    }

    void lookWhereToPark()
    {
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
                    switch (tag.id)
                    {
                        case ID_TAG_OF_INTEREST1:
                        default:
                            parkIn = ParkIn.CENTER;
                            break;
                        case ID_TAG_OF_INTEREST2:
                            parkIn = ParkIn.RIGHT;
                            break;
                        case ID_TAG_OF_INTEREST3:
                            parkIn = ParkIn.LEFT;
                            break;
                    }
                }

                switch (parkIn)
                {
                    case LEFT:
                        telemetry.addLine("Parking In Left Zone!");
                        break;
                    case CENTER:
                        telemetry.addLine("Parking In Center Zone(May not see tag)!");
                        break;
                    case RIGHT:
                        telemetry.addLine("Parking In Right Zone!");
                        break;
                }

            }

            telemetry.update();
            sleep(20);
        }
    }

    void initCamera()
    {
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
    }

}