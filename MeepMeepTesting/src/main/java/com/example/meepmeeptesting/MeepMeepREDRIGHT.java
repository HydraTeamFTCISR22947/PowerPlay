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
    public static double startPosX = 36, startPosY = -66, startPosAngle = 180;
    public static double startConeStrafe1 = 58.8, startConeStrafe2 = 19.5, startConeForward = 4.5;
    public static double intakePose1X = 35, intakePose1Y = -20.3, intakePose1Angle = 180;
    public static double intakePose2X = 62.5, intakePose2Y = -15, intakePose2Angle = 180;
    public static double posCone1X = 41.5, posCone1Y = -18;
    public static double posCone2X = 35.5, posCone2Y = -29.5, posCone2Angle = 180;
    public static double DELIVERY_WAIT_TIME = .25, RELEASE_WAIT_TIME = .33, INTAKE_WAIT_TIME = .8, ELEVATOR_WAIT_TIME = .25;
    public static double PARK_ASSIST = 20, TARGET_ZONE = 24;


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(70), new TranslationalVelocityConstraint(70)));
        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(60);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)


                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(130), Math.toRadians(130), 13.23)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPosX, startPosY, Math.toRadians(startPosAngle)))
                                .strafeRight(startConeStrafe1, velConstraint, accelConstraint)
                                .strafeLeft(startConeStrafe2)

                                .lineTo(new Vector2d(intakePose1X, intakePose1Y))
                                .splineToLinearHeading(new Pose2d(intakePose2X, intakePose2Y, Math.toRadians(intakePose2Angle)), Math.toRadians(0))
                                .waitSeconds(INTAKE_WAIT_TIME)

                                .setReversed(true)
                                .lineTo(new Vector2d(intakePose1X, intakePose1Y))
                                .splineToLinearHeading(new Pose2d(intakePose2X, intakePose2Y, Math.toRadians(intakePose2Angle)), Math.toRadians(0))

                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}