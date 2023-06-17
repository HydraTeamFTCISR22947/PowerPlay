package com.example.meepmeep;

import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepBlueRight {
    public static void main(String[] args)
    {
        MeepMeep meepMeep = new MeepMeep(550);

        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(60), new TranslationalVelocityConstraint(60)));
        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(30);

    }
}