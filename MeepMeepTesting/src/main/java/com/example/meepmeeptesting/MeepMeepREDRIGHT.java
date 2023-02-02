package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepREDRIGHT {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

         double START_POSE_X = 36, START_POSE_Y = -72, START_POSE_ANGLE = 90;

        double START_CONE_DELIVERY_X = 36, START_CONE_DELIVERY_Y = -12, START_CONE_DELIVERY_ANGLE = 127;
        double CONE_DELIVERY_POSE_X = 36, CONE_DELIVERY_POSE_Y = -12, CONE_DELIVERY_POSE_ANGLE = 127;
        double CONE_INTAKE_POSE_X = 56, CONE_INTAKE_POSE_Y = -12, CONE_INTAKE_POSE_ANGLE = 180;
        double NEAR_CONE_INTAKE_POSE_X = 46, NEAR_CONE_INTAKE_POSE_Y = -14;
        //Need to put actual coordinates

        double DELIVERY_WAIT_TIME = 1, INTAKE_WAIT_TIME = 2;
        double INTAKE_OFFSET = 1;// move closer to cone stack each cycle ( check this irl )

        Pose2d startPose = new Pose2d(START_POSE_X, START_POSE_Y, Math.toRadians(START_POSE_ANGLE));

        Pose2d firstCycleBarPose = new Pose2d(START_CONE_DELIVERY_X, START_CONE_DELIVERY_Y, Math.toRadians(START_CONE_DELIVERY_ANGLE));
        Vector2d firstCycleBarVector = new Vector2d(START_CONE_DELIVERY_X, START_CONE_DELIVERY_Y);
        Pose2d secondCycleBarPose = new Pose2d(CONE_DELIVERY_POSE_X, CONE_DELIVERY_POSE_Y, Math.toRadians(CONE_DELIVERY_POSE_ANGLE));

        Pose2d nearConeStackPose = new Pose2d(NEAR_CONE_INTAKE_POSE_X, NEAR_CONE_INTAKE_POSE_Y, Math.toRadians(CONE_INTAKE_POSE_ANGLE));
        Pose2d coneStackPose = new Pose2d(CONE_INTAKE_POSE_X, CONE_INTAKE_POSE_Y, Math.toRadians(CONE_INTAKE_POSE_ANGLE));


        Pose2d coneStackPoseSecond = new Pose2d(CONE_INTAKE_POSE_X + INTAKE_OFFSET, CONE_INTAKE_POSE_Y, Math.toRadians(CONE_INTAKE_POSE_ANGLE));
        Pose2d coneStackPoseThird = new Pose2d(CONE_INTAKE_POSE_X + 2 * INTAKE_OFFSET, CONE_INTAKE_POSE_Y, Math.toRadians(CONE_INTAKE_POSE_ANGLE));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(130), Math.toRadians(130), 13.23)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                        /* First Cycle*/
                                .lineToLinearHeading(firstCycleBarPose)//delivery
                                .waitSeconds(2)
                            //    .splineToLinearHeading(new Pose2d(56, -12,Math.toRadians(CONE_INTAKE_POSE_ANGLE)), Math.toRadians(0))


                                .splineToLinearHeading(new Pose2d(44, -12, Math.toRadians(180)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(56, -12, Math.toRadians(180)))

                                //   .splineTo(new Vector2d(56, -12), Math.toRadians(0))
                               // .lineToLinearHeading(new Pose2d(56, -11.90, Math.toRadians(0)))

                                /*Second Cycle*/
                    .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}