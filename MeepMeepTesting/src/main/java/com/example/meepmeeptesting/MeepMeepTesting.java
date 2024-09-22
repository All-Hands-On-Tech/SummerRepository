package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                //bot is 18x18 box
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

//        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(25, -62, Math.toRadians(90)))
//                .strafeTo(new Vector2d(10, -34))
//                //score specimen
//                .strafeTo(new Vector2d(20, -45))
//                .setTangent(-45)
//                .splineToLinearHeading(new Pose2d(45, -14, Math.toRadians(90)), Math.toRadians(-70))
//                .strafeTo(new Vector2d(45, -59))
//                .setTangent(90)
//                .splineToLinearHeading(new Pose2d(55.00, -11, Math.toRadians(90)), Math.toRadians(-90))
//                .strafeTo(new Vector2d(55, -60))
//                .setTangent(90)
//                .splineToSplineHeading(new Pose2d(62, -24.10, Math.toRadians(180)), Math.toRadians(0))
//                .strafeTo(new Vector2d(62, -62))
//                .build());

        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(-14, -62, Math.toRadians(90)))
                .strafeTo(new Vector2d(-10, -34))
//               //score specimen
                .strafeTo(new Vector2d(-26, -40))
                .setTangent(110)
                .splineToLinearHeading(new Pose2d(-45, -14, Math.toRadians(90)), Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(-58, -58, Math.toRadians(45)), Math.toRadians(225))
                .strafeTo(new Vector2d( -40, -27))
                .splineToLinearHeading(new Pose2d(-57.00, -14, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-58, -58, Math.toRadians(60)), Math.toRadians(240))
                .strafeTo(new Vector2d( -50, -27))
                .splineToLinearHeading(new Pose2d(-62.00, -24, Math.toRadians(0)), Math.toRadians(180))
                .strafeTo(new Vector2d(-62, -55))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(mySecondBot)
                .start();
    }
}