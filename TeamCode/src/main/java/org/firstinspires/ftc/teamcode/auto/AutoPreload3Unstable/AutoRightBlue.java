package org.firstinspires.ftc.teamcode.auto.AutoPreload3Unstable;

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
import org.firstinspires.ftc.teamcode.auto.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;


@Autonomous(name = "Auto Right Blue", group = "auto")
public class AutoRightBlue extends LinearOpMode {


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

    @Override
    public void runOpMode() throws InterruptedException {
        if(PoseStorage.useCamera)
        {
            initCamera();
        }

        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(PoseStorage.startPosX, PoseStorage.startPosY, Math.toRadians(PoseStorage.startPosAngle));
        autoCommands = new AutoCommands(hardwareMap);
        clawServo = new ClawServo(hardwareMap);
        elevatorSystem = new ElevatorSystem(hardwareMap);
        transferSystem = new TransferSystem(hardwareMap);


        drivetrain.setPoseEstimate(startPose);

        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(70), new TranslationalVelocityConstraint(70)));

        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL);

        TrajectorySequence preload = drivetrain.trajectorySequenceBuilder(startPose)
                .strafeRight(PoseStorage.startConeStrafe1, velConstraint, accelConstraint)
                .strafeLeft(PoseStorage.startConeStrafe2, velConstraint, accelConstraint)
                .forward(PoseStorage.startConeForward, velConstraint, accelConstraint)
                .waitSeconds(PoseStorage.DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCommands.goDownToReleaseCone())
                .waitSeconds(PoseStorage.ALMOST_RELEASE_TIME)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(PoseStorage.RELEASE_WAIT_TIME)
                .build();

        TrajectorySequence cycle1 = drivetrain.trajectorySequenceBuilder(preload.end())
                .lineTo(new Vector2d(-PoseStorage.intakePose1XFirstCone, -PoseStorage.intakePose1YFirstCone))
                .addTemporalMarker(autoCommands.intakeFirstCone())
                .splineToLinearHeading(new Pose2d(-PoseStorage.intakePose2XFirstCone, -PoseStorage.intakePose2YFirstCone, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(PoseStorage.BACK_WAIT_TIME)
                .back(PoseStorage.backIntakeOffset)
                .waitSeconds(PoseStorage.INTAKE_WAIT_TIME)
                .addTemporalMarker(autoCommands.catchCone())
                .waitSeconds(PoseStorage.ELEVATOR_WAIT_TIME)
                .addTemporalMarker(autoCommands.elevatorIntake())
                .build();

        TrajectorySequence place1 = drivetrain.trajectorySequenceBuilder(cycle1.end())
                .lineTo(new Vector2d(-PoseStorage.posCone1X, -PoseStorage.posCone1Y))
                .addTemporalMarker(autoCommands.readyToRelease())
                .splineTo(new Vector2d(-PoseStorage.posCone2XFirstCone, -PoseStorage.posCone2YFirstCone), Math.toRadians(PoseStorage.posConeAngle-180))
                .waitSeconds(PoseStorage.DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCommands.goDownToReleaseCone())
                .waitSeconds(PoseStorage.ALMOST_RELEASE_TIME)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(PoseStorage.RELEASE_WAIT_TIME)
                .build();


        TrajectorySequence cycle2 = drivetrain.trajectorySequenceBuilder(place1.end())
                .splineToLinearHeading(new Pose2d(-PoseStorage.intakePoseCycleXSecondCone, -PoseStorage.intakePose1YSecondCone, Math.toRadians(0)), Math.toRadians(180))
                .addTemporalMarker(autoCommands.intakeSecondCone())
                .lineTo(new Vector2d(-PoseStorage.intakePose2XSecondCone, -PoseStorage.intakePose2YSecondCone))
                .waitSeconds(PoseStorage.BACK_WAIT_TIME)
                .back(PoseStorage.backIntakeOffset)
                .waitSeconds(PoseStorage.INTAKE_WAIT_TIME+0.16)
                .addTemporalMarker(autoCommands.catchCone())
                .waitSeconds(PoseStorage.ELEVATOR_WAIT_TIME)
                .addTemporalMarker(autoCommands.elevatorIntake())
                .build();

        TrajectorySequence place2 = drivetrain.trajectorySequenceBuilder(cycle2.end())
                .lineTo(new Vector2d(-PoseStorage.posCone1X, -PoseStorage.posCone1Y))
                .addTemporalMarker(autoCommands.readyToRelease())
                .splineTo(new Vector2d(-PoseStorage.posCone2XSecondCone, -PoseStorage.posCone2YSecondCone), Math.toRadians(PoseStorage.posConeAngle - 180))
                .waitSeconds(PoseStorage.DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCommands.goDownToReleaseCone())
                .waitSeconds(PoseStorage.ALMOST_RELEASE_TIME)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(PoseStorage.RELEASE_WAIT_TIME)
                .build();

        TrajectorySequence cycle3 = drivetrain.trajectorySequenceBuilder(place2.end())
                .splineToLinearHeading(new Pose2d(-PoseStorage.intakePoseCycleXThirdCone, -PoseStorage.intakePose1YThirdCone, Math.toRadians(0)), Math.toRadians(180))
                .addTemporalMarker(autoCommands.intakeThirdCone())
                .lineTo(new Vector2d(-PoseStorage.intakePose2XThirdCone, -PoseStorage.intakePose2YThirdCone))
                .waitSeconds(PoseStorage.BACK_WAIT_TIME)
                .back(PoseStorage.backIntakeOffset)
                .waitSeconds(PoseStorage.INTAKE_WAIT_TIME)
                .addTemporalMarker(autoCommands.catchCone())
                .waitSeconds(PoseStorage.ELEVATOR_WAIT_TIME)
                .addTemporalMarker(autoCommands.elevatorIntake())
                .build();

        TrajectorySequence place3 = drivetrain.trajectorySequenceBuilder(cycle3.end())
                .lineTo(new Vector2d(-PoseStorage.posCone1X, -PoseStorage.posCone1Y))
                .addTemporalMarker(autoCommands.readyToRelease())
                .splineTo(new Vector2d(-PoseStorage.posCone2XThirdCone, -PoseStorage.posCone2YThirdCone), Math.toRadians(PoseStorage.posConeAngle - 180))
                .waitSeconds(PoseStorage.DELIVERY_WAIT_TIME)
                .addTemporalMarker(autoCommands.goDownToReleaseCone())
                .waitSeconds(PoseStorage.ALMOST_RELEASE_TIME)
                .addTemporalMarker(autoCommands.releaseCone())
                .waitSeconds(PoseStorage.RELEASE_WAIT_TIME)
                .build();


        TrajectorySequence park3 = drivetrain.trajectorySequenceBuilder(place3.end())
                .back(PoseStorage.GO_TO_PARK_HELPER,velConstraint,accelConstraint)
                .addTemporalMarker(autoCommands.readyToRelease())
                .addTemporalMarker(autoCommands.reset())
                .lineToLinearHeading(new Pose2d(-PoseStorage.parkPoseX, -PoseStorage.parkPoseY, Math.toRadians(0)),velConstraint,accelConstraint)
                .back(PoseStorage.TARGET_ZONE)
                .build();

        TrajectorySequence park2 = drivetrain.trajectorySequenceBuilder(place3.end())
                .back(PoseStorage.GO_TO_PARK_HELPER,velConstraint,accelConstraint)
                .addTemporalMarker(autoCommands.readyToRelease())
                .addTemporalMarker(autoCommands.reset())
                .lineToLinearHeading(new Pose2d(-PoseStorage.parkPoseX, -PoseStorage.parkPoseY, Math.toRadians(0)),velConstraint,accelConstraint)
                .build();

        TrajectorySequence park1 = drivetrain.trajectorySequenceBuilder(place3.end())
                .back(PoseStorage.GO_TO_PARK_HELPER,velConstraint,accelConstraint)
                .addTemporalMarker(autoCommands.readyToRelease())
                .addTemporalMarker(autoCommands.reset())
                .lineToLinearHeading(new Pose2d(-PoseStorage.parkPoseX, -PoseStorage.parkPoseY, Math.toRadians(0)),velConstraint,accelConstraint)
                .forward(PoseStorage.TARGET_ZONE)
                .build();

        if (isStopRequested()) {return;}

        clawServo.closeClaw();

        if(PoseStorage.useCamera)
        {
            lookWhereToPark();
        }

        waitForStart();

        elevatorSystem.goToPos(ElevatorSystem.BASE_HEIGHT);
        elevatorSystem.midRod();
        transferSystem.highPos();

        drivetrain.followTrajectorySequence(preload);

        drivetrain.followTrajectorySequence(cycle1);
        drivetrain.followTrajectorySequence(place1);

        drivetrain.followTrajectorySequence(cycle2);
        drivetrain.followTrajectorySequence(place2);

        drivetrain.followTrajectorySequence(cycle3);
        drivetrain.followTrajectorySequence(place3);

        if(PoseStorage.useCamera)
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