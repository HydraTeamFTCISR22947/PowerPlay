package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBLUERIGHT {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(550);

        double startPoseX = -36, startPoseY = 72, startPoseAngle = 270;

        double startConeDeliveryPoseX = -36, startConeDeliveryPoseY = 0, startConeDeliveryAngle = 0;
        double coneDeliveryPoseX = -32, coneDeliveryPoseY = 12, coneDeliveryAngle = 307;
        double coneIntakePoseX = -56, coneIntakePoseY = 12, coneIntakeAngle = 0;

        double offsetBetweenIntakes = 1;
        double DELIVERY_WAIT_TIME = 2, INTAKE_WAIT_TIME = 4;

        Pose2d startPose = new Pose2d(startPoseX,startPoseY, Math.toRadians(startPoseAngle));

        Pose2d firstCycleBarPose = new Pose2d(startConeDeliveryPoseX,startConeDeliveryPoseY, Math.toRadians(startConeDeliveryAngle));
        Pose2d secondCycleBarPose = new Pose2d(coneDeliveryPoseX,coneDeliveryPoseY,Math.toRadians(coneDeliveryAngle));
        Pose2d coneStackPose = new Pose2d(coneIntakePoseX,coneIntakePoseY, Math.toRadians(coneIntakeAngle));
        Pose2d coneStackPoseSecond = new Pose2d(coneIntakePoseX-offsetBetweenIntakes,coneIntakePoseY,Math.toRadians(coneIntakeAngle));
        Pose2d coneStackPoseThird = new Pose2d(coneIntakePoseX-2*offsetBetweenIntakes,coneIntakePoseY,Math.toRadians(coneIntakeAngle));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(130), Math.toRadians(130), 13.23)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                //First Cycle
                                .lineToLinearHeading(firstCycleBarPose)//delivery
                                .waitSeconds(DELIVERY_WAIT_TIME)

                                //Second cycle
                                .splineToLinearHeading(coneStackPose,Math.toRadians(startPoseAngle))//intake
                                .waitSeconds(INTAKE_WAIT_TIME)

                                .lineToLinearHeading(secondCycleBarPose)//delivery
                                .waitSeconds(DELIVERY_WAIT_TIME)

                                //Third cycle
                                .lineToLinearHeading(coneStackPoseSecond)//intake
                                .waitSeconds(INTAKE_WAIT_TIME)

                                .lineToLinearHeading(secondCycleBarPose)//delivery
                                .waitSeconds(DELIVERY_WAIT_TIME)

                                //Fourth cycle
                                .lineToLinearHeading(coneStackPoseThird)//intake
                                .waitSeconds(INTAKE_WAIT_TIME)

                                .lineToLinearHeading(secondCycleBarPose)//delivery
                                .waitSeconds(DELIVERY_WAIT_TIME)


                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}