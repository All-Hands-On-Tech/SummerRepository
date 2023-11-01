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

}
