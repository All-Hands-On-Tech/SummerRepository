package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        double trackWidth = 15.0;
        double fieldWidth = 140; //field is 144 inches wide
        double tileWidth = 24.0;

        Pose2d startPose = new Pose2d(60, -35, Math.toRadians(180));

        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 60, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineToLinearHeading(new Pose2d(35, -33, Math.toRadians(90)), Math.toRadians(90))
                                .waitSeconds(5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(59, -35, Math.toRadians(90)), Math.toRadians(90))
                                .strafeTo(new Vector2d(59, 47))
                                .build()
                );

        RoadRunnerBotEntity mid = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(33, -35), Math.toRadians(180))
                                .waitSeconds(5)
                                .strafeTo(new Vector2d(59, -35))
                                .setReversed(true)
                                .strafeTo(new Vector2d(59, 47))
                                .build()
                );

        RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(35, -37, Math.toRadians(-90)))
                                .waitSeconds(5)
                                .strafeTo(new Vector2d(59, -37))
                                .setReversed(true)
                                .strafeTo(new Vector2d(59, 47))
                                .build()
                );


        Image img = null;
        try { img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/centerstageField.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                .addEntity(left)
                .addEntity(mid)
                .addEntity(right)
                .start();;

                //official background
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(centerStageBot)
//                .start();
    }
}