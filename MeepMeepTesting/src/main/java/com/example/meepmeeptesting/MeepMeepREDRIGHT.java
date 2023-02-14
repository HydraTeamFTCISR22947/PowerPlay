package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.util.Arrays;

public class MeepMeepREDRIGHT {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        double startPosX = 36, startPosY = -65.9, startPosAngle = 180;

        double startConeStrafe1 = 54.8, startConeStrafe2 = 16.05, startConeForward = 5.15;


        double backIntakeOffset  = 13;


        double intakePose1XFirstCone = 35, intakePose1YFirstCone = -19.8;
        double intakePose2XFirstCone = 48.7, intakePose2YFirstCone = -15.3;


        double intakePoseCycleXSecondCone = 38, intakePose1YSecondCone = -15.7;
        double intakePose2XSecondCone = 50, intakePose2YSecondCone = -15.4;


        double intakePoseCycleXThirdCone = 38, intakePose1YThirdCone = -15.7;
        double intakePose2XThirdCone = 51, intakePose2YThirdCone = -15;


        double intakeAngle = 180;


        double posCone1X = 40, posCone1Y = -15, posConeAngle = 225;


        double posCone2XFirstCone = 30.45, posCone2YFirstCone = -18.05;


        double posCone2XSecondCone = 30.6, posCone2YSecondCone = -17.4;


        double posCone2XThirdCone = 31.95, posCone2YThirdCone = -17.2;


        double parkPoseX = 39, parkPoseY = -16.5, parkPoseAngle = 180;


        double TARGET_ZONE = 20, GO_TO_PARK_HELPER = 8;


        double BACK_WAIT_TIME = 0.1, DELIVERY_WAIT_TIME = .35, RELEASE_WAIT_TIME = .33;
        double ALMOST_RELEASE_TIME = 0.1, INTAKE_WAIT_TIME = .1, ELEVATOR_WAIT_TIME = .5;
        
        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(70), new TranslationalVelocityConstraint(70)));
        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(60);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(130), Math.toRadians(130), 13.23)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPosX, startPosY, Math.toRadians(startPosAngle)))
                                .strafeRight(startConeStrafe1, velConstraint, accelConstraint)
                                .strafeLeft(startConeStrafe2, velConstraint, accelConstraint)
                                .forward(startConeForward, velConstraint, accelConstraint)
                              /*
                                .waitSeconds(DELIVERY_WAIT_TIME)
                                .waitSeconds(ALMOST_RELEASE_TIME)
                                .waitSeconds(RELEASE_WAIT_TIME)

                                .lineTo(new Vector2d(intakePose1XFirstCone, intakePose1YFirstCone))
                                .splineToLinearHeading(new Pose2d(intakePose2XFirstCone, intakePose2YFirstCone, Math.toRadians(intakeAngle)), Math.toRadians(0))
                                .waitSeconds(BACK_WAIT_TIME)
                                .back(backIntakeOffset)
                                .waitSeconds(INTAKE_WAIT_TIME)
                                .waitSeconds(ELEVATOR_WAIT_TIME)

                                .lineTo(new Vector2d(posCone1X, posCone1Y))
                                .splineTo(new Vector2d(posCone2XFirstCone, posCone2YFirstCone), Math.toRadians(posConeAngle))
                                .waitSeconds(DELIVERY_WAIT_TIME)
                                .waitSeconds(ALMOST_RELEASE_TIME)
                                .waitSeconds(RELEASE_WAIT_TIME)

                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(intakePoseCycleXSecondCone, intakePose1YSecondCone, Math.toRadians(intakeAngle)), Math.toRadians(0))
                                .lineTo(new Vector2d(intakePose2XSecondCone, intakePose2YSecondCone))
                                .waitSeconds(BACK_WAIT_TIME)
                                .back(backIntakeOffset)
                                .waitSeconds(INTAKE_WAIT_TIME)
                                .waitSeconds(ELEVATOR_WAIT_TIME)

                                .lineTo(new Vector2d(posCone1X, posCone1Y))
                                .splineTo(new Vector2d(posCone2XSecondCone, posCone2YSecondCone), Math.toRadians(posConeAngle))
                                .waitSeconds(DELIVERY_WAIT_TIME)
                                .waitSeconds(ALMOST_RELEASE_TIME)
                                .waitSeconds(RELEASE_WAIT_TIME)

                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(intakePoseCycleXThirdCone, intakePose1YThirdCone, Math.toRadians(intakeAngle)), Math.toRadians(0))
                                .lineTo(new Vector2d(intakePose2XThirdCone, intakePose2YThirdCone))
                                .waitSeconds(BACK_WAIT_TIME)
                                .back(backIntakeOffset)
                                .waitSeconds(INTAKE_WAIT_TIME)
                                .waitSeconds(ELEVATOR_WAIT_TIME)

                                .lineTo(new Vector2d(posCone1X, posCone1Y))
                                .splineTo(new Vector2d(posCone2XThirdCone, posCone2YThirdCone), Math.toRadians(posConeAngle))
                                .waitSeconds(DELIVERY_WAIT_TIME)
                                .waitSeconds(ALMOST_RELEASE_TIME)
                                .waitSeconds(RELEASE_WAIT_TIME)

                                .back(GO_TO_PARK_HELPER,velConstraint,accelConstraint)

                               */
                                .lineToLinearHeading(new Pose2d(parkPoseX, parkPoseY, Math.toRadians(180)),velConstraint,accelConstraint)
                                //.forward(20) //ZONE 1
                                //.back(20) //ZONE 3

                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}