package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Test {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(130), Math.toRadians(130), 13.23)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(27, -20, Math.toRadians(-135)))
                                .lineTo(new Vector2d(30, -16))
                                .splineToSplineHeading(new Pose2d(55, -12.5, Math.toRadians(180)), Math.toRadians(-5))
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}