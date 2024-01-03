package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.Vector;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    private static Pose2d RED_BACKSTAGE_START_POSE = new Pose2d(59.5, 14, Math.toRadians(180));
    private static Pose2d RED_LANDING_ZONE_START_POSE = new Pose2d(59.5, -38, Math.toRadians(180));
    private static Pose2d BLUE_BACKSTAGE_START_POSE = new Pose2d(-59.5, 14, Math.toRadians(0));
    private static Pose2d BLUE_LANDING_ZONE_START_POSE = new Pose2d(-59.5, -38, Math.toRadians(0));


    private static Pose2d BLUE_END_POSE = new Pose2d(-34, 38, Math.toRadians(90));

    private static Pose2d RED_END_POSE = new Pose2d(34, 38, Math.toRadians(90));
    public static void main(String[] args) {

        double trackWidth = 15.0;
        double fieldWidth = 140; //field is 144 inches wide
        double tileWidth = 24.0;

        Pose2d startPose = new Pose2d(-60, -35, Math.toRadians(0));

        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 60, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineToLinearHeading(new Pose2d(-35, -33, Math.toRadians(90)), Math.toRadians(90))
                                .waitSeconds(5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-59, -35, Math.toRadians(90)), Math.toRadians(90))
                                .strafeTo(new Vector2d(-59, 47))
                                .build()
                );

        RoadRunnerBotEntity mid = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(-33, -35), Math.toRadians(0))
                                .waitSeconds(5)
                                .strafeTo(new Vector2d(-59, -35))
                                .setReversed(true)
                                .strafeTo(new Vector2d(-59, 47))
                                .build()
                );

        RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(-35, -37, Math.toRadians(-90)))
                                .waitSeconds(5)
                                .strafeTo(new Vector2d(-35, -35))
                                .strafeTo(new Vector2d(-59, -35))
                                .setReversed(true)
                                .strafeTo(new Vector2d(-59, 47))
                                .build()
                );

        //CHOPPER PUSH BOT TRAJECTORIES

        //BLUE

        //BLUE BACKSTAGE

        //blue backstage left
        RoadRunnerBotEntity BlueBackstageLeftTrajectoryChopperPush0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
                        .splineToLinearHeading(new Pose2d(-30, 33, Math.toRadians(90)), Math.toRadians(-20))
                        .build()
                );
        RoadRunnerBotEntity BlueBackstageLeftTrajectoryChopperPush1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-30, 33, Math.toRadians(90)))
                            .lineToLinearHeading(new Pose2d(-56, 33, Math.toRadians(90)))
                            .waitSeconds(1)
                            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(90)), Math.toRadians(90))
                            .build()
                );


        //Blue backstage center
        RoadRunnerBotEntity BlueBackstageCenterTrajectoryChopperPush0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
                            .splineToLinearHeading(new Pose2d(-13, 11, Math.toRadians(0)), Math.toRadians(-90))
                            .build()
                );

        RoadRunnerBotEntity BlueBackstageCenterTrajectoryChopperPush1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-13, 11, Math.toRadians(0)))
                            .back(20)
                            .waitSeconds(1)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
                            .build()
                );


        //Blue backstage right

        RoadRunnerBotEntity BlueBackstageRightTrajectoryChopperPush0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
                            .splineToLinearHeading(new Pose2d(-35, 10, Math.toRadians(90)), Math.toRadians(-90))
                            .build()
                );

        RoadRunnerBotEntity BlueBackstageRightTrajectoryChopperPush1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 10, Math.toRadians(90)))
                            .forward(20)
                            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(90)), Math.toRadians(90))
                            .build()
                );

        //BLUE LANDING ZONE


        //Blue landing zone right
        RoadRunnerBotEntity BlueLandingZoneRightTrajectoryChopperPush0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
                            .splineToLinearHeading(new Pose2d(-35, -57, Math.toRadians(-90)), Math.toRadians(0))
                            .build()
                );

        RoadRunnerBotEntity BlueLandingZoneRightTrajectoryChopperPush1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -57, Math.toRadians(-90)))
                            .lineToLinearHeading(new Pose2d(-13, -57, Math.toRadians(0)))
                            .lineToLinearHeading(new Pose2d(-10, 50, Math.toRadians(0)))
                            .build()
                );


        //Blue landing zone center

        RoadRunnerBotEntity BlueLandingZoneCenterTrajectoryChopperPush0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
                                .splineToLinearHeading(new Pose2d(-13, -35, Math.toRadians(0)), Math.toRadians(90))
                                .build()
                );

        RoadRunnerBotEntity BlueLandingZoneCenterTrajectoryChopperPush1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-13, -35, Math.toRadians(0)))
                            .forward(3)
                            .lineToLinearHeading(new Pose2d(-10, 50, Math.toRadians(0)))
                            .build()
                );


        //Red landing zone left
        RoadRunnerBotEntity RedLandingZoneLeftTrajectoryChopperPush0 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
                            .splineToLinearHeading(new Pose2d(35, -34, Math.toRadians(-90)), Math.toRadians(90))
                            .build()
                );

        RoadRunnerBotEntity RedLandingZoneLeftTrajectoryChopperPush1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -34, Math.toRadians(-90)))
                            .forward(15)
                            .strafeRight(21)
                            .lineToLinearHeading(new Pose2d(12, 50, Math.toRadians(-90)))
                            .build()
                );



        //Chopper with slides

        //Blue landingzone

        RoadRunnerBotEntity BlueLandingZoneLeftChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 60, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
                                .splineToLinearHeading(new Pose2d(-35, -33, Math.toRadians(90)), Math.toRadians(90))
                                .waitSeconds(5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-12, -35, Math.toRadians(90)), Math.toRadians(90))
                                .strafeTo(new Vector2d(-12, 30))
                                .strafeTo(new Vector2d(BLUE_END_POSE.getX(), BLUE_END_POSE.getY()))
                                .build()
                );

        RoadRunnerBotEntity BlueLandingZoneCenterChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
                                .splineTo(new Vector2d(-33, -35), Math.toRadians(0))
                                .waitSeconds(5)
                                .strafeTo(new Vector2d(-36, -48))
                                .splineToLinearHeading(new Pose2d(-12, -35, Math.toRadians(90)), Math.toRadians(90))
                                .strafeTo(new Vector2d(-12, 30))
                                .strafeTo(new Vector2d(BLUE_END_POSE.getX(), BLUE_END_POSE.getY()))
                                .build()
                );

        RoadRunnerBotEntity BlueLandingZoneRightChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
                                .lineToLinearHeading(new Pose2d(-35, -37, Math.toRadians(-90)))
                                .waitSeconds(5)
                                .lineToLinearHeading(new Pose2d(-12, -37, Math.toRadians(180)))
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(-12, 30, Math.toRadians(90)))
                                .strafeTo(new Vector2d(BLUE_END_POSE.getX(), BLUE_END_POSE.getY()))
                                .build()
                );

        //blue backstage

        RoadRunnerBotEntity BlueBackstageLeftChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
                                .splineToLinearHeading(new Pose2d(-40, 21, Math.toRadians(0)), Math.toRadians(-20))
                                .setReversed(true)
                                .strafeTo(new Vector2d(-53, 21))
                                .lineToLinearHeading(new Pose2d(BLUE_END_POSE.getX(), BLUE_END_POSE.getY(), Math.toRadians(90)))
                                .build()
                );

        RoadRunnerBotEntity BlueBackstageCenterChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
                                .splineToLinearHeading(new Pose2d(-33, 15, Math.toRadians(0)), Math.toRadians(0))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-40, 9, Math.toRadians(90)), Math.toRadians(-135))
                                .strafeTo(new Vector2d(BLUE_END_POSE.getX(), BLUE_END_POSE.getY()))
                                .build()
                );

        RoadRunnerBotEntity BlueBackstageRightChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
                                .splineToLinearHeading(new Pose2d(-33, 10, Math.toRadians(-90)), Math.toRadians(-90))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-40, 15, Math.toRadians(-90)), Math.toRadians(-135))
                                .lineToLinearHeading(new Pose2d(BLUE_END_POSE.getX(), BLUE_END_POSE.getY(), Math.toRadians(90)))
                                .waitSeconds(5)
                                .build()
                );


        //red landingzone

        RoadRunnerBotEntity RedLandingZoneRightChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 60, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
                                .splineToLinearHeading(new Pose2d(35, -33, Math.toRadians(90)), Math.toRadians(90))
                                .waitSeconds(5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(12, -35, Math.toRadians(90)), Math.toRadians(90))
                                .strafeTo(new Vector2d(12, 30))
                                .strafeTo(new Vector2d(RED_END_POSE.getX(), RED_END_POSE.getY()))
                                .build()
                );

        RoadRunnerBotEntity RedLandingZoneCenterChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
                                .splineTo(new Vector2d(33, -35), Math.toRadians(180))
                                .waitSeconds(5)
                                .strafeTo(new Vector2d(39, -48))
                                .splineToLinearHeading(new Pose2d(12, -35, Math.toRadians(90)), Math.toRadians(90))
                                .strafeTo(new Vector2d(12, 30))
                                .strafeTo(new Vector2d(RED_END_POSE.getX(), RED_END_POSE.getY()))
                                .build()
                );

        RoadRunnerBotEntity RedLandingZoneLeftChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
                                .lineToLinearHeading(new Pose2d(35, -37, Math.toRadians(-90)))
                                .waitSeconds(5)
                                .lineToLinearHeading(new Pose2d(12, -37, Math.toRadians(0) -1e-6))
                                .turn(Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(12, 30, Math.toRadians(90)))
                                .strafeTo(new Vector2d(RED_END_POSE.getX(), RED_END_POSE.getY()))
                                .build()
                );

        //red backstage

        RoadRunnerBotEntity RedBackstageLeftChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
                                .splineToLinearHeading(new Pose2d(32, 10, Math.toRadians(-90)), Math.toRadians(-90))
                                .setReversed(true)
                                .strafeTo(new Vector2d(53, 21))
                                .lineToLinearHeading(new Pose2d(RED_END_POSE.getX(), RED_END_POSE.getY(), Math.toRadians(90)))
                                .build()
                );

        RoadRunnerBotEntity RedBackstageCenterChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
                                .splineToLinearHeading(new Pose2d(34, 12, Math.toRadians(180)), Math.toRadians(180))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(43, 7, Math.toRadians(90)), Math.toRadians(-135))
                                .strafeTo(new Vector2d(RED_END_POSE.getX(), RED_END_POSE.getY()))
                                .build()
                );

        RoadRunnerBotEntity RedBackstageRightChopper = new DefaultBotBuilder(meepMeep)
                .setConstraints(39.2, 30, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
                                .splineToLinearHeading(new Pose2d(33, 10, Math.toRadians(-90)), Math.toRadians(-90))
                                .setReversed(true)
                                .strafeTo(new Vector2d(33, 15))
                                .lineToLinearHeading(new Pose2d(RED_END_POSE.getX(), RED_END_POSE.getY(), Math.toRadians(90)))
                                .build()
                );

        Image img = null;
        MeepMeep.Background background = null;
        boolean imgExists = false;
        try { img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/centerstageField.png")); imgExists = true;}
        catch (IOException e) {background = MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK; imgExists = false;}

        if(imgExists) {
            meepMeep.setBackground(img)
//                .addEntity(left)
//                .addEntity(mid)
//                .addEntity(right)
                    .addEntity(BlueBackstageRightChopper)
                    .start();
        }else {
            meepMeep.setBackground(background)
//                .addEntity(left)
//                .addEntity(mid)
//                .addEntity(right)
                    .addEntity(BlueBackstageRightChopper)
                    .start();
        }

                //official background
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(centerStageBot)
//                .start();
    }
}