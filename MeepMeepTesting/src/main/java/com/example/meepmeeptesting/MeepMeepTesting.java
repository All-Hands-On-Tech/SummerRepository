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
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(20, 60, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                                .lineTo(new Vector2d(24, 0))
                                .splineToSplineHeading(new Pose2d(0, -24, Math.toRadians(0)), Math.toRadians(180))
                                .build()
                );

        RoadRunnerBotEntity centerStageBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(fieldWidth/2 - trackWidth/2, fieldWidth/2 - trackWidth/2, Math.toRadians(90)))
                                .lineTo(new Vector2d(fieldWidth/2 - trackWidth/2, -(fieldWidth/2 - trackWidth/2) ))
                                .lineTo(new Vector2d(fieldWidth/2 - trackWidth/2 - tileWidth, -(fieldWidth/2 - trackWidth/2) ))
                                .lineTo(new Vector2d(fieldWidth/2 - trackWidth/2, -(fieldWidth/2 - trackWidth/2) ))
                                .lineTo(new Vector2d(fieldWidth/2 - trackWidth/2, fieldWidth/2 - trackWidth/2))
                                .build()
                );


        Image img = null;
        try { img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/centerstageField.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img).addEntity(centerStageBot).start();;

                //official background
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(centerStageBot)
//                .start();
    }
}