
        package com.example.meepmeeptesting;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
        import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
        import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
        import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
        import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
        import com.noahbres.meepmeep.MeepMeep;
        import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
        import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

        import java.util.Arrays;

public class AutoRightBlueHigh {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        double startPosX = 36, startPosY = 65.9, startPosAngle = 180;
        double startConeStrafe1 = -48.8, startConeStrafe2 = 12, startConeForward = 3.2;
        double placeConeX = 10.5, placeConeY = 13.5, placeConeAngle = 135;
        double firstIntakeX = -60, firstIntakeY = 12, intakeAngle = 0;



        double backIntakeOffset = 13;


        double intakePose1XFirstCone = 36, intakePose1YFirstCone = 19;
        double intakePose2XFirstCone = 46.7, intakePose2YFirstCone = 15.6;


        double intakePoseCycleXSecondCone = 38, intakePose1YSecondCone = -15.7;
        double intakePose2XSecondCone = 50, intakePose2YSecondCone = -15.4;


        double intakePoseCycleXThirdCone = 38, intakePose1YThirdCone = -15.7;
        double intakePose2XThirdCone = 51, intakePose2YThirdCone = -15;


        double posCone1X = 40, posCone1Y = -15, posConeAngle = 225;


        double posCone2XFirstCone = 31.5, posCone2YFirstCone = -18.05;


        double posCone2XSecondCone = 32, posCone2YSecondCone = -17.4;


        double posCone2XThirdCone = 32.5, posCone2YThirdCone = -17.2;


        double parkPoseX = 39.3, parkPoseY = -20, parkPoseAngle = 180;


        double TARGET_ZONE = 20, GO_TO_PARK_HELPER = 3;


        double BACK_WAIT_TIME = 0.1, DELIVERY_WAIT_TIME = .25, RELEASE_WAIT_TIME = .33;
        double ALMOST_RELEASE_TIME = 0.1, INTAKE_WAIT_TIME = .1, ELEVATOR_WAIT_TIME = .5;

        final int ID_TAG_OF_INTEREST1 = 0, ID_TAG_OF_INTEREST2 = 1, ID_TAG_OF_INTEREST3 = 2; // Tags from the 36h11 family


        Pose2d startPose = new Pose2d(startPosX, startPosY, Math.toRadians(startPosAngle));
        Pose2d highRodPose = new Pose2d(placeConeX, placeConeY, Math.toRadians(placeConeAngle));
        Pose2d intakePose = new Pose2d(firstIntakeX, firstIntakeY, Math.toRadians(intakeAngle));

        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(70), new TranslationalVelocityConstraint(70)));
        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(60);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)


                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(130), Math.toRadians(130), 13.23)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)



                        //Getting to position
                        .strafeRight(startConeStrafe1)
                        .lineToLinearHeading(highRodPose, velConstraint, accelConstraint)
                        //placing
                        .waitSeconds(DELIVERY_WAIT_TIME)
                        .waitSeconds(ALMOST_RELEASE_TIME)
                        //.addTemporalMarker(autoCommands.releaseCone())
                        .waitSeconds(RELEASE_WAIT_TIME)

                        //TrajectorySequence firstIntake = drivetrain.trajectorySequenceBuilder(placePreload.end())
                        //     .
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

                        .back(GO_TO_PARK_HELPER, velConstraint, accelConstraint)
                        .lineToLinearHeading(new Pose2d(parkPoseX, parkPoseY, Math.toRadians(parkPoseAngle)), velConstraint, accelConstraint)
                        .back(TARGET_ZONE)


                        .back(GO_TO_PARK_HELPER, velConstraint, accelConstraint)
                        .lineToLinearHeading(new Pose2d(parkPoseX, parkPoseY, Math.toRadians(parkPoseAngle)), velConstraint, accelConstraint)


                        .back(GO_TO_PARK_HELPER, velConstraint, accelConstraint)
                        .lineToLinearHeading(new Pose2d(parkPoseX, parkPoseY, Math.toRadians(parkPoseAngle)), velConstraint, accelConstraint)
                        .forward(TARGET_ZONE)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }

}