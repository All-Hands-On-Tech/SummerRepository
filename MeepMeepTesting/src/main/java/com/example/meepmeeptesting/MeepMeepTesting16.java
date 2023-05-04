package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting16 {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(80, 1000, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-50, 40, 0))
                                .splineTo(new Vector2d(-50,50),Math.toRadians(180))
                                .splineTo(new Vector2d(-50,50.1),Math.toRadians(0))
                                .splineTo(new Vector2d(-50,60),Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-55, 60, Math.toRadians(0)))
                                .lineTo(new Vector2d(-55,40))
                                .lineTo(new Vector2d(-42,40))
                                .lineTo(new Vector2d(-42,52))
                                .lineToLinearHeading(new Pose2d(-42, 46, Math.toRadians(90)))
                                .splineTo(new Vector2d(-36,47),Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(-36, 47.1, Math.toRadians(90)))
                                .splineTo(new Vector2d(-42,48),Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(-42, 40, Math.toRadians(0)))
                                .lineTo(new Vector2d(-31,40))
                                .lineTo(new Vector2d(-31,50))
                                .splineTo(new Vector2d(-31,53),Math.toRadians(180))
                                .splineTo(new Vector2d(-31,50),Math.toRadians(0))
                                .lineTo(new Vector2d(-31,40))

                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}