package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

public class AutonomousTrajectories {

    public SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    private Pose2d RED_BACKSTAGE_START_POSE = new Pose2d(60, 14, Math.toRadians(180));
    private Pose2d RED_LANDING_ZONE_START_POSE = new Pose2d(60, -38, Math.toRadians(180));
    private Pose2d BLUE_BACKSTAGE_START_POSE = new Pose2d(-60, 14, Math.toRadians(0));
    private Pose2d BLUE_LANDING_ZONE_START_POSE = new Pose2d(-60, -38, Math.toRadians(0));


    //BLUE BACKSTAGE
    public TrajectorySequence BlueBackstageLeftTrajectory = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(-36, 23), Math.toRadians(0))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    public TrajectorySequence BlueBackstageCenterTrajectory = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(-30, 11), Math.toRadians(0))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    public TrajectorySequence BlueBackstageRightTrajectory = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineToLinearHeading(new Pose2d(-32, 10, Math.toRadians(-90)), Math.toRadians(-90))
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();


//BLUE LANDING ZONE
    public TrajectorySequence BlueLandingZoneRightTrajectory = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineToConstantHeading(new Vector2d(-38, -47), Math.toRadians(0))
            .back(7)
            .strafeLeft(12.5)
            .forward(33)
            .lineToLinearHeading(new Pose2d(-8, 50, Math.toRadians(0)))
            .build();

    public TrajectorySequence BlueLandingZoneCenterTrajectory = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineTo(new Vector2d(-31, -35), Math.toRadians(0))
            .back(5)
            .strafeRight(15)
            .forward(25)
            .lineToLinearHeading(new Pose2d(-10, 50, Math.toRadians(0)))
            .build();

    public TrajectorySequence BlueLandingZoneLeftTrajectory = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineToLinearHeading(new Pose2d(-30, -34, Math.toRadians(90)), Math.toRadians(90))
            .back(5)
            .strafeRight(18)
            .lineToLinearHeading(new Pose2d(-12, 50, Math.toRadians(90)))
            .build();




    //RED BACKSTAGE
    public TrajectorySequence RedBackstageLeftTrajectory = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .splineToLinearHeading(new Pose2d(29, 10, Math.toRadians(-90)), Math.toRadians(-90))
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    public TrajectorySequence RedBackstageCenterTrajectory = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(30, 11), Math.toRadians(180))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    public TrajectorySequence RedBackstageRightTrajectory = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(36, 23), Math.toRadians(180))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();


    //RED LANDING ZONE
    public TrajectorySequence RedLandingZoneLeftTrajectory = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .splineToConstantHeading(new Vector2d(38, -47), Math.toRadians(180))
            .back(7)
            .strafeRight(12.5)
            .forward(33)
            .lineToLinearHeading(new Pose2d(8, 50, Math.toRadians(180)))
            .build();

    public TrajectorySequence RedLandingZoneCenterTrajectory = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .splineTo(new Vector2d(31, -35), Math.toRadians(180))
            .back(5)
            .strafeLeft(15)
            .forward(27)
            .turn(Math.toRadians(90))
            .back(95)
            .build();

    public TrajectorySequence RedLandingZoneRightTrajectory = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .splineToLinearHeading(new Pose2d(30, -34, Math.toRadians(90)), Math.toRadians(90))
            .back(5)
            .strafeLeft(18)
            .lineToLinearHeading(new Pose2d(12, 50, Math.toRadians(90)))
            .build();

    //CHOPPER PUSH BOT TRAJECTORIES

    //BLUE

    //BLUE BACKSTAGE

    //blue backstage left
    public TrajectorySequence BlueBackstageLeftTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(-36, 33), Math.toRadians(0))
            .build();
    public TrajectorySequence BlueBackstageLeftTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-36, 33, Math.toRadians(0)))
            .back(20)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    //Blue backstage center
    public TrajectorySequence BlueBackstageCenterTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(-20, 11), Math.toRadians(0))
            .build();

    public TrajectorySequence BlueBackstageCenterTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-20, 11, Math.toRadians(0)))
            .back(20)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();


    //Blue backstage right

    public TrajectorySequence BlueBackstageRightTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineToLinearHeading(new Pose2d(-32, 0, Math.toRadians(-90)), Math.toRadians(-90))
            .build();

    public TrajectorySequence BlueBackstageRightTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-32, 0, Math.toRadians(-90)))
            .back(20)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    //BLUE LANDING ZONE


    //Blue landing zone right
    public TrajectorySequence BlueLandingZoneRightTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineToConstantHeading(new Vector2d(-38, -57), Math.toRadians(0))
            .build();

    public TrajectorySequence BlueLandingZoneRightTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-38, -57, Math.toRadians(0)))
            .back(17)
            .strafeLeft(12.5)
            .forward(33)
            .lineToLinearHeading(new Pose2d(-8, 50, Math.toRadians(0)))
            .build();


    //Blue landing zone center

    public TrajectorySequence BlueLandingZoneCenterTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineTo(new Vector2d(-21, -35), Math.toRadians(0))
            .build();

    public TrajectorySequence BlueLandingZoneCenterTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-21, -35, Math.toRadians(0)))
            .back(15)
            .strafeRight(15)
            .forward(25)
            .lineToLinearHeading(new Pose2d(-10, 50, Math.toRadians(0)))
            .build();


    //Blue landing zone left
    public TrajectorySequence BlueLandingZoneLeftTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineToLinearHeading(new Pose2d(-30, -24, Math.toRadians(90)), Math.toRadians(90))
            .build();

    public TrajectorySequence BlueLandingZoneLeftTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-30, -24, Math.toRadians(90)))
            .back(15)
            .strafeRight(18)
            .lineToLinearHeading(new Pose2d(-12, 50, Math.toRadians(90)))
            .build();

    //RED

    //RED BACKSTAGE

    //Red backstage right

    public TrajectorySequence RedBackstageRightTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .setReversed(true)
            .splineTo(new Vector2d(36, 23), Math.toRadians(0))
            .build();
    public TrajectorySequence RedBackstageRightTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(36, 23, Math.toRadians(0)))
            .setReversed(true)
            .back(10)
            .waitSeconds(1)
            .setReversed(false)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    //Red backstage center
    public TrajectorySequence RedBackstageCenterTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .setReversed(true)
            .splineTo(new Vector2d(30, 11), Math.toRadians(0))
            .build();

    public TrajectorySequence RedBackstageCenterTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(30, 11, Math.toRadians(0)))
            .setReversed(true)
            .back(10)
            .waitSeconds(1)
            .setReversed(false)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();


    //red backstage right

    public TrajectorySequence RedBackstageLeftTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(32, 10, Math.toRadians(-90)), Math.toRadians(-90))
            .build();

    public TrajectorySequence RedBackstageLeftTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(32, 10, Math.toRadians(-90)))
            .setReversed(false)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    //RED LANDING ZONE


    //red landing zone right
    public TrajectorySequence RedLandingZoneLeftTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .setReversed(true)
            .splineToConstantHeading(new Vector2d(38, -47), Math.toRadians(0))
            .build();

    public TrajectorySequence RedLandingZoneLeftTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(38, -47, Math.toRadians(0)))
            .setReversed(true)
            .back(7)
            .strafeLeft(12.5)
            .forward(33)
            .lineToLinearHeading(new Pose2d(8, 50, Math.toRadians(0)))
            .build();


    //Blue landing zone center

    public TrajectorySequence RedLandingZoneCenterTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .setReversed(true)
            .splineTo(new Vector2d(31, -35), Math.toRadians(0))
            .build();

    public TrajectorySequence RedLandingZoneCenterTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(31, -35, Math.toRadians(0)))
            .setReversed(true)
            .back(5)
            .strafeRight(15)
            .forward(25)
            .lineToLinearHeading(new Pose2d(10, 50, Math.toRadians(0)))
            .build();


    //Red landing zone left
    public TrajectorySequence RedLandingZoneRightTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(30, -34, Math.toRadians(90)), Math.toRadians(90))
            .build();

    public TrajectorySequence RedLandingZoneRightTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(30, -34, Math.toRadians(90)))
            .setReversed(true)
            .back(5)
            .strafeRight(18)
            .lineToLinearHeading(new Pose2d(12, 50, Math.toRadians(90)))
            .build();
}
