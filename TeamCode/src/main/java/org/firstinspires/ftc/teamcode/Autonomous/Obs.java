package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Obs OneFish", group = "Autonomous")
public class Obs extends LinearOpMode {
    Delivery delivery = null;
    Intake intake = null;

    private static final int VERTICAL_INTAKE_POS = -330;
    private static final int CLAW_FLOOR = 0;
    private static final int CLAW_COLLECT = 800;
    private static final int CLAW_SCORE = 1650;
    private static final int CLAW_HIGH_RUNG = 2200;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(15.2, -62, Math.toRadians(90)));

        Action trajToScoreFirstSpecimen;
        Action trajToCollectFirstSample;
        Action trajToCollectSecondSample;
        Action trajToCollectSecondSpecimen;
        Action trajToScoreSecondSpecimen;
        Action trajToPrepareThirdSpecimen;
        Action trajToCollectThirdSpecimen;
        Action trajToScoreThirdSpecimen;
        Action trajToPark;

        trajToScoreFirstSpecimen = drive.actionBuilder(new Pose2d(15.2, -62, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(6, -34), Math.toRadians(90))
                .build();

        trajToCollectFirstSample = drive.actionBuilder(new Pose2d(6, -34, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(48, -45), Math.toRadians(0))
                .build();

        trajToCollectSecondSample = drive.actionBuilder(new Pose2d(48, -45, Math.toRadians(90)))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(58, -45), Math.toRadians(0))
                .build();

        trajToCollectSecondSpecimen = drive.actionBuilder(new Pose2d(58, -45, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(58, -59), Math.toRadians(90))
                .build();

        trajToScoreSecondSpecimen = drive.actionBuilder(new Pose2d(58, -59, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(6, -34), Math.toRadians(90))
                .build();

        trajToPrepareThirdSpecimen = drive.actionBuilder(new Pose2d(6, -34, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(45, -47), Math.toRadians(-90))
                .build();

        trajToCollectThirdSpecimen = drive.actionBuilder(new Pose2d(45, -47, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(45, -59), Math.toRadians(-90))
                .build();

        trajToScoreThirdSpecimen = drive.actionBuilder(new Pose2d(45, -59, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(6, -34), Math.toRadians(90))
                .build();

        trajToPark = drive.actionBuilder(new Pose2d(6, -34, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineTo(new Vector2d(45, -55), Math.toRadians(-90))
                .build();

        intake.brakePitch();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(trajToScoreFirstSpecimen);
        //Add code to score a specimen
        Actions.runBlocking(trajToCollectFirstSample);
        //Add code to intake and outtake a sample
        Actions.runBlocking(trajToCollectSecondSample);
        //Add code to intake and outtake sample
        Actions.runBlocking(trajToCollectSecondSpecimen);
        //Add code to collect a specimen
        Actions.runBlocking(trajToScoreSecondSpecimen);
        //Add code to score a specimen
        Actions.runBlocking(trajToPrepareThirdSpecimen);
        //Add wait to help human player align to robot
        Actions.runBlocking(trajToCollectThirdSpecimen);
        //Add code to collect a specimen
        Actions.runBlocking(trajToScoreThirdSpecimen);
        //Add code to score a specimen
        Actions.runBlocking(trajToPark);

    }
}