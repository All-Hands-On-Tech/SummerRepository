package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Competition.BlueBackstage;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

public class AutonomousTrajectories {

    LinearOpMode linearOpMode;

    SampleMecanumDrive drive;

//    public AutonomousTrajectories(BlueBackstage blueBackstage) {
//    }
//
//    public AutonomousTrajectories() {
//
//    }


    public AutonomousTrajectories(LinearOpMode l) {
        linearOpMode = l;
        drive = new SampleMecanumDrive(l.hardwareMap);
        Initialize();
    }

    private Pose2d RED_BACKSTAGE_START_POSE = new Pose2d(60, 14, Math.toRadians(180));
    private Pose2d RED_LANDING_ZONE_START_POSE = new Pose2d(60, -38, Math.toRadians(180));
    private Pose2d BLUE_BACKSTAGE_START_POSE = new Pose2d(-60, 14, Math.toRadians(0));
    private Pose2d BLUE_LANDING_ZONE_START_POSE = new Pose2d(-60, -38, Math.toRadians(0));

    public TrajectorySequence BlueBackstageLeftTrajectory;
    public TrajectorySequence BlueBackstageCenterTrajectory;
    public TrajectorySequence BlueBackstageRightTrajectory;
    public TrajectorySequence BlueLandingZoneRightTrajectory;
    public TrajectorySequence BlueLandingZoneCenterTrajectory;
    public TrajectorySequence BlueLandingZoneLeftTrajectory;
    public TrajectorySequence RedBackstageLeftTrajectory;
    public TrajectorySequence RedBackstageCenterTrajectory;
    public TrajectorySequence RedBackstageRightTrajectory;
    public TrajectorySequence RedLandingZoneLeftTrajectory;
    public TrajectorySequence RedLandingZoneCenterTrajectory;
    public TrajectorySequence RedLandingZoneRightTrajectory;
    public TrajectorySequence BlueBackstageLeftTrajectoryChopperPush0;
    public TrajectorySequence BlueBackstageLeftTrajectoryChopperPush1;
    public TrajectorySequence BlueBackstageCenterTrajectoryChopperPush0;
    public TrajectorySequence BlueBackstageCenterTrajectoryChopperPush1;
    public TrajectorySequence BlueBackstageRightTrajectoryChopperPush0;
    public TrajectorySequence BlueBackstageRightTrajectoryChopperPush1;
    public TrajectorySequence BlueLandingZoneRightTrajectoryChopperPush0;
    public TrajectorySequence BlueLandingZoneRightTrajectoryChopperPush1;
    public TrajectorySequence BlueLandingZoneCenterTrajectoryChopperPush0;
    public TrajectorySequence BlueLandingZoneCenterTrajectoryChopperPush1;
    public TrajectorySequence BlueLandingZoneLeftTrajectoryChopperPush0;
    public TrajectorySequence BlueLandingZoneLeftTrajectoryChopperPush1;
    public TrajectorySequence RedBackstageRightTrajectoryChopperPush0;
    public TrajectorySequence RedBackstageRightTrajectoryChopperPush1;
    public TrajectorySequence RedBackstageCenterTrajectoryChopperPush0;
    public TrajectorySequence RedBackstageCenterTrajectoryChopperPush1;
    public TrajectorySequence RedBackstageLeftTrajectoryChopperPush0;
    public TrajectorySequence RedBackstageLeftTrajectoryChopperPush1;
    public TrajectorySequence RedLandingZoneLeftTrajectoryChopperPush0;
    public TrajectorySequence RedLandingZoneLeftTrajectoryChopperPush1;
    public TrajectorySequence RedLandingZoneCenterTrajectoryChopperPush0;
    public TrajectorySequence RedLandingZoneCenterTrajectoryChopperPush1;
    public TrajectorySequence RedLandingZoneRightTrajectoryChopperPush0;
    public TrajectorySequence RedLandingZoneRightTrajectoryChopperPush1;

    private void Initialize(){

    //BLUE BACKSTAGE
     BlueBackstageLeftTrajectory = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(-36, 23), Math.toRadians(0))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    BlueBackstageCenterTrajectory = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(-30, 11), Math.toRadians(0))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    BlueBackstageRightTrajectory = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineToLinearHeading(new Pose2d(-32, 10, Math.toRadians(-90)), Math.toRadians(-90))
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();


    //BLUE LANDING ZONE
    BlueLandingZoneRightTrajectory = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineToConstantHeading(new Vector2d(-38, -47), Math.toRadians(0))
            .back(7)
            .strafeLeft(12.5)
            .forward(33)
            .lineToLinearHeading(new Pose2d(-8, 50, Math.toRadians(0)))
            .build();

    BlueLandingZoneCenterTrajectory = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineTo(new Vector2d(-31, -35), Math.toRadians(0))
            .back(5)
            .strafeRight(15)
            .forward(25)
            .lineToLinearHeading(new Pose2d(-10, 50, Math.toRadians(0)))
            .build();

    BlueLandingZoneLeftTrajectory = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineToLinearHeading(new Pose2d(-30, -34, Math.toRadians(90)), Math.toRadians(90))
            .back(5)
            .strafeRight(18)
            .lineToLinearHeading(new Pose2d(-12, 50, Math.toRadians(90)))
            .build();


    //RED BACKSTAGE
    RedBackstageLeftTrajectory = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .splineToLinearHeading(new Pose2d(29, 10, Math.toRadians(-90)), Math.toRadians(-90))
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    RedBackstageCenterTrajectory = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(30, 11), Math.toRadians(180))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    RedBackstageRightTrajectory = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(36, 23), Math.toRadians(180))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();


    //RED LANDING ZONE
    RedLandingZoneLeftTrajectory = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .splineToConstantHeading(new Vector2d(38, -47), Math.toRadians(180))
            .back(7)
            .strafeRight(12.5)
            .forward(33)
            .lineToLinearHeading(new Pose2d(8, 50, Math.toRadians(180)))
            .build();

    RedLandingZoneCenterTrajectory = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .splineTo(new Vector2d(31, -35), Math.toRadians(180))
            .back(5)
            .strafeLeft(15)
            .forward(27)
            .turn(Math.toRadians(90))
            .back(95)
            .build();

    RedLandingZoneRightTrajectory = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .splineToLinearHeading(new Pose2d(30, -34, Math.toRadians(90)), Math.toRadians(90))
            .back(5)
            .strafeLeft(18)
            .lineToLinearHeading(new Pose2d(12, 50, Math.toRadians(90)))
            .build();

    //CHOPPER PUSH BOT TRAJECTORIES

    //BLUE

    //BLUE BACKSTAGE

    //blue backstage left
    BlueBackstageLeftTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineToLinearHeading(new Pose2d(-30, 33, Math.toRadians(90)), Math.toRadians(-20))
            .build();
    BlueBackstageLeftTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-13, 11, Math.toRadians(0)))
            .back(20)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    //Blue backstage center
    BlueBackstageCenterTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(-20, 11), Math.toRadians(0))
            .build();

    BlueBackstageCenterTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-20, 11, Math.toRadians(0)))
            .back(20)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();


    //Blue backstage right

    BlueBackstageRightTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineToLinearHeading(new Pose2d(-35, 10, Math.toRadians(90)), Math.toRadians(-90))
            .build();

   BlueBackstageRightTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-35, 10, Math.toRadians(90)))
            .forward(20)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(90)), Math.toRadians(90))
            .build();

    //BLUE LANDING ZONE


    //Blue landing zone right
    BlueLandingZoneRightTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineToConstantHeading(new Vector2d(-38, -57), Math.toRadians(0))
            .build();

    BlueLandingZoneRightTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-38, -57, Math.toRadians(0)))
            .back(17)
            .strafeLeft(12.5)
            .forward(33)
            .lineToLinearHeading(new Pose2d(-8, 50, Math.toRadians(0)))
            .build();


    //Blue landing zone center

   BlueLandingZoneCenterTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineTo(new Vector2d(-21, -35), Math.toRadians(0))
            .build();

     BlueLandingZoneCenterTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-21, -35, Math.toRadians(0)))
            .back(15)
            .strafeRight(15)
            .forward(25)
            .lineToLinearHeading(new Pose2d(-10, 50, Math.toRadians(0)))
            .build();


    //Blue landing zone left
    BlueLandingZoneLeftTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineToLinearHeading(new Pose2d(-30, -24, Math.toRadians(90)), Math.toRadians(90))
            .build();

    BlueLandingZoneLeftTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(-30, -24, Math.toRadians(90)))
            .back(15)
            .strafeRight(18)
            .lineToLinearHeading(new Pose2d(-12, 50, Math.toRadians(90)))
            .build();

    //RED

    //RED BACKSTAGE

    //Red backstage right

    RedBackstageRightTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .setReversed(true)
            .splineTo(new Vector2d(36, 23), Math.toRadians(0))
            .build();
    RedBackstageRightTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(36, 23, Math.toRadians(0)))
            .setReversed(true)
            .back(10)
            .waitSeconds(1)
            .setReversed(false)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    //Red backstage center
    RedBackstageCenterTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .setReversed(true)
            .splineTo(new Vector2d(30, 11), Math.toRadians(0))
            .build();

    RedBackstageCenterTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(30, 11, Math.toRadians(0)))
            .setReversed(true)
            .back(10)
            .waitSeconds(1)
            .setReversed(false)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();


    //red backstage right

    RedBackstageLeftTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(32, 10, Math.toRadians(-90)), Math.toRadians(-90))
            .build();

    RedBackstageLeftTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(32, 10, Math.toRadians(-90)))
            .setReversed(false)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    //RED LANDING ZONE


    //red landing zone right
     RedLandingZoneLeftTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .setReversed(true)
            .splineToConstantHeading(new Vector2d(38, -47), Math.toRadians(0))
            .build();

    RedLandingZoneLeftTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(38, -47, Math.toRadians(0)))
            .setReversed(true)
            .back(7)
            .strafeLeft(12.5)
            .forward(33)
            .lineToLinearHeading(new Pose2d(8, 50, Math.toRadians(0)))
            .build();


    //Blue landing zone center

    RedLandingZoneCenterTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .setReversed(true)
            .splineTo(new Vector2d(31, -35), Math.toRadians(0))
            .build();

    RedLandingZoneCenterTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(31, -35, Math.toRadians(0)))
            .setReversed(true)
            .back(5)
            .strafeRight(15)
            .forward(25)
            .lineToLinearHeading(new Pose2d(10, 50, Math.toRadians(0)))
            .build();


    //Red landing zone left
    RedLandingZoneRightTrajectoryChopperPush0 = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(30, -34, Math.toRadians(90)), Math.toRadians(90))
            .build();

    RedLandingZoneRightTrajectoryChopperPush1 = drive.trajectorySequenceBuilder(new Pose2d(30, -34, Math.toRadians(90)))
            .setReversed(true)
            .back(5)
            .strafeRight(18)
            .lineToLinearHeading(new Pose2d(12, 50, Math.toRadians(90)))
            .build();

}
}
