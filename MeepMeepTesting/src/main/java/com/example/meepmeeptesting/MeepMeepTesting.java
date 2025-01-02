package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8.875, -61.71875, Math.toRadians(270)))
                .strafeTo(new Vector2d(8.875, -37.71875)) // Go to Chamber
                .strafeTo(new Vector2d(37.375, -37.71875)) // Move to the right
                .strafeTo(new Vector2d(37.375, -14.71875)) // Move Forward
                .strafeTo(new Vector2d(45.875, -14.71875)) // Move to Right
                .strafeTo(new Vector2d(45.875, -53.46875)) // Move to Wall
                .splineToLinearHeading(new Pose2d(47, -40, Math.toRadians(90)), Math.toRadians(90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}