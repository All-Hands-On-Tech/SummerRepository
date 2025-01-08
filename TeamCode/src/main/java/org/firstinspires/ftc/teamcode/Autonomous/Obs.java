package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.OneFishIntake;
import org.firstinspires.ftc.teamcode.OneFishSampleDelivery;
import org.firstinspires.ftc.teamcode.OneFishSpecimenDelivery;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Obs OneFish", group = "Autonomous")
public class Obs extends LinearOpMode {
    OneFishSpecimenDelivery specimenDelivery = null;
    OneFishIntake intake = null;
    OneFishSampleDelivery sampleDelivery = null;

    DrivetrainFunctions driveTrain = null;

    private static final int VERTICAL_INTAKE_POS = -330;
    private static final int CLAW_FLOOR = 0;
    private static final int CLAW_COLLECT = 800;
    private static final int CLAW_SCORE = 1650;
    private static final int CLAW_HIGH_RUNG = 2200;

    private boolean imuInitialized = false;

    @Override
    public void runOpMode() {
        driveTrain = new DrivetrainFunctions(this, true);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(15.2, -62, Math.toRadians(90)));


        specimenDelivery = new OneFishSpecimenDelivery(this);
        intake = new OneFishIntake(this);
        sampleDelivery = new OneFishSampleDelivery(this, true);

        Action trajToScoreFirstSpecimen;
        Action trajToCollectFirstSample;
        Action trajToOuttakeFirstSample;
        Action trajToCollectSecondSample;
        Action trajToOuttakeSecondSample;
        Action trajToCollectSecondSpecimen;
        Action trajToScoreSecondSpecimen;
        Action trajToPrepareThirdSpecimen;
        Action trajToCollectThirdSpecimen;
        Action trajToScoreThirdSpecimen;
        Action trajToPark;

        trajToScoreFirstSpecimen = drive.actionBuilder(new Pose2d(15.2, -62, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(0, -34), Math.toRadians(90))
                .build();

        trajToCollectFirstSample = drive.actionBuilder(new Pose2d(0, -34, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(35, -39, Math.toRadians(23)), Math.toRadians(90))
                .build();

        trajToOuttakeFirstSample = drive.actionBuilder(new Pose2d(35, -39, Math.toRadians(23)))
                .turnTo(Math.toRadians(-75))
                .build();

        trajToCollectSecondSample = drive.actionBuilder(new Pose2d(35, -39, Math.toRadians(-75)))
                .turnTo(Math.toRadians(28))
                .build();

        trajToOuttakeSecondSample = drive.actionBuilder(new Pose2d(35, -39, Math.toRadians(28)))
                .turnTo(Math.toRadians(-75))
                .build();

        trajToCollectSecondSpecimen = drive.actionBuilder(new Pose2d(35, -39, Math.toRadians(-75)))
                .turnTo(Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(35, -56), Math.toRadians(90))
                .build();

        trajToScoreSecondSpecimen = drive.actionBuilder(new Pose2d(35, -56, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-2, -38), Math.toRadians(90))
                .build();

//        trajToPrepareThirdSpecimen = drive.actionBuilder(new Pose2d(-2, -38, Math.toRadians(90)))
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(45, -47), Math.toRadians(-90))
//                .build();

        trajToCollectThirdSpecimen = drive.actionBuilder(new Pose2d(-2, -38, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -65), Math.toRadians(90))
                .build();

        trajToScoreThirdSpecimen = drive.actionBuilder(new Pose2d(35, -65, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-6, -38), Math.toRadians(90))
                .build();

        trajToPark = drive.actionBuilder(new Pose2d(6, -34, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineTo(new Vector2d(45, -55), Math.toRadians(-90))
                .build();

        specimenDelivery.clawClose();
        specimenDelivery.setPivotPosition(0.6);

        imuInitialized = driveTrain.imu.initialize(driveTrain.myIMUparameters);

        waitForStart();

        if (isStopRequested()) return;

        intake.pitchUp();

//        Actions.runBlocking(trajToScoreFirstSpecimen);
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(trajToScoreFirstSpecimen, specimenDelivery.PrepScoreSpecimenAction(0, 1250)),
                new SequentialAction(driveTrain.DriveForTimeAction(500, 0.35, 0.0),
                                     specimenDelivery.ScoreSpecimenAction(1000))
        ));
        //Add code to score a specimen
        Actions.runBlocking(trajToCollectFirstSample);


        Actions.runBlocking(new ParallelAction(
                intake.RunToLengthAction(450, 1000),
                intake.SpinIntakeAction(-1,500)
        ));
        intake.pitchDown();
        Actions.runBlocking(intake.SpinIntakeAction(-1,250));

        Actions.runBlocking(new ParallelAction(
                intake.SpinIntakeAction(-1,500),
                driveTrain.TurnToAngleAction(Math.toRadians(-120), 500),
                intake.RunToLengthAction(1200, 1000)
        ));

        Actions.runBlocking(intake.SpinIntakeAction(0.5,250));
        intake.pitchUp();


        Actions.runBlocking(new ParallelAction(
                trajToCollectSecondSample,
                intake.SpinIntakeAction(0.5,500),
                intake.RunToLengthAction(1200, 100)
        ));
//        //Add code to intake and outtake sample


        intake.pitchDown();
        Actions.runBlocking(intake.SpinIntakeAction(-1,250));

        Actions.runBlocking(new ParallelAction(
                intake.SpinIntakeAction(-1,500),
                driveTrain.TurnToAngleAction(Math.toRadians(-120), 500),
                intake.RunToLengthAction(1200, 1000)
        ));

        Actions.runBlocking(intake.SpinIntakeAction(0.5,250));

        intake.pitchUp();

        specimenDelivery.pitchToIntake();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(trajToCollectSecondSpecimen, intake.RunToLengthAction(0, 1000), intake.SpinIntakeAction(0.5,500)),
                driveTrain.DriveForTimeAction(500, -0.25, 0.0),
                specimenDelivery.PrepScoreSpecimenAction(500, 1250)
        ));
//        //Add code to collect a specimen
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(trajToScoreSecondSpecimen, specimenDelivery.PrepScoreSpecimenAction(0, 1000)),
                new SequentialAction(driveTrain.DriveForTimeAction(500, 0.35, 0.0), specimenDelivery.ScoreSpecimenAction(1000))
        ));
//        //Add code to score a specimen

        specimenDelivery.pitchToIntake();

//        //Add wait to help human player align to robot
        Actions.runBlocking(new SequentialAction(
                trajToCollectThirdSpecimen,
                driveTrain.DriveForTimeAction(500, -0.25, 0.0),
                specimenDelivery.PrepScoreSpecimenAction(500, 1100)
        ));
//        //Add code to collect a specimen
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(trajToScoreThirdSpecimen, specimenDelivery.PrepScoreSpecimenAction(0, 2000)),
                new SequentialAction(driveTrain.DriveForTimeAction(400, 0.25, 0.0), specimenDelivery.ScoreSpecimenAction(1000))
        ));
//        //Add code to score a specimen
//        Actions.runBlocking(trajToPark);

    }
}