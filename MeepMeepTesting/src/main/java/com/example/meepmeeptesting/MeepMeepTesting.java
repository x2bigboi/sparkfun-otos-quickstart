package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -60, Math.toRadians(90)))
                .strafeTo(new Vector2d(0,-34))
                .setTangent(Math.toRadians(-35))
                .splineToConstantHeading(new Vector2d(30, -40),Math.toRadians(20))
                .splineToConstantHeading(new Vector2d(45.3, -8),Math.toRadians(0))
                .turn(Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(40, -60))
                .setTangent(Math.toRadians(-35))
                .splineToConstantHeading(new Vector2d(54, -10),Math.toRadians(0))
                .strafeTo(new Vector2d(56,-57))
                        .splineToConstantHeading(new Vector2d(60, -8),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(69, -60),Math.toRadians(0))
                        .strafeTo(new Vector2d(40, -58))
                .waitSeconds(0.75)
                        .turn(Math.toRadians(180))
                        .strafeTo(new Vector2d(0, -34))
                .waitSeconds(0.75)
                        .splineTo(new Vector2d(40,-58), Math.toRadians(0))
                                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(40, -58))
                .waitSeconds(0.75)
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(0, -34))
                .waitSeconds(0.75)
                .splineTo(new Vector2d(40,-58), Math.toRadians(0))
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(40, -58))
                .waitSeconds(0.75)
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(0, -34))
                .waitSeconds(0.75)
                .splineTo(new Vector2d(40,-58), Math.toRadians(0))
                .turn(Math.toRadians(-90))



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}