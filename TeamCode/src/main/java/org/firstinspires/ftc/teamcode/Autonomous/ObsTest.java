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
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.SparkFunOTOSDrive;

@Config
@Autonomous(name = "Obs Meet1", group = "Testing")
public class ObsTest extends LinearOpMode {
    Delivery delivery = null;
    Intake intake = null;

    @Override
    public void runOpMode() {
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(15.2, -62, Math.toRadians(90)));
        delivery = new Delivery(this, false);
        intake = new Intake(this);

        delivery.clawClose();

        Action trajToScoreFirstSample;
        Action trajToCollectSamples;
        Action trajToCollectSecondSample;
        Action trajToScoreSecondSample1;
        Action trajToScoreSecondSample2;
        Action trajToCollectThirdSample1;
        Action trajToCollectThirdSample2;
        Action trajToScoreThirdSample;
        Action trajToPark;



        trajToScoreFirstSample = drive.actionBuilder(drive.pose)
                //Scores pre set specimin
                .strafeTo(new Vector2d(9, -33))
                /*score specimin*/
                .build();

        trajToCollectSamples = drive.actionBuilder(new Pose2d(9, -33, Math.toRadians(90)))
                //Brings two samples to observation zone
                .strafeTo(new Vector2d(25, -40))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(36, -15,Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(new Pose2d(44, -58, Math.toRadians(90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(42, -12, Math.toRadians(180)), Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(55, -58), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(39, -49), Math.toRadians(-90))
                .turnTo(Math.toRadians(-80))
                .build();

        trajToCollectSecondSample = drive.actionBuilder(new Pose2d(39, -49, Math.toRadians(-90)))
                //Scores a second specimin
                /*sleep*/
                .strafeTo(new Vector2d(40.5, -61.5))
                .build();

        trajToScoreSecondSample1 = drive.actionBuilder(new Pose2d(40.5, -61.5, Math.toRadians(-90)))
                /*grab specimin*/
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(6, -50, Math.toRadians(95)), Math.toRadians(90))
                /*score specimin*/
                .build();

        trajToScoreSecondSample2 = drive.actionBuilder(new Pose2d(6, -50, Math.toRadians(95)))
                /*grab specimin*/
                .setTangent(Math.toRadians(90))
                .strafeTo(new Vector2d(6, -33))
                /*score specimin*/
                .build();

        trajToCollectThirdSample1 = drive.actionBuilder(new Pose2d(6, -34, Math.toRadians(90)))
                //This scores a third specimin, add if theres time
                .strafeTo(new Vector2d(10, -40))
                .splineToLinearHeading(new Pose2d(36, -49, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        trajToCollectThirdSample2 = drive.actionBuilder(new Pose2d(36, -49, Math.toRadians(-90)))
                .strafeTo(new Vector2d(36, -60))
                .build();
                /*grab specimin*/

        trajToScoreThirdSample = drive.actionBuilder(new Pose2d(36, -60, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(6, -34,Math.toRadians(90)), Math.toRadians(90))
                /*score specimin*/
                .build();

        trajToPark = drive.actionBuilder(new Pose2d(6, -34,Math.toRadians(90)))
                //Returns to observation zone
                .setTangent(Math.toRadians(-90))
                .splineTo(new Vector2d(40, -57), Math.toRadians(-45))
                .build();

        intake.brakePitch();

        waitForStart();

        intake.setTargetLengthServo(0.2);

        //This code scores preloaded specimen
        Actions.runBlocking(
                new ParallelAction(
                        trajToScoreFirstSample,
                        delivery.SlideToHeightAction(2100),
                        intake.pitchToAngleAction(-250)
                )
        );
//        intake.updateAngle();
        delivery.clawToTarget(1650, 5);
        delivery.clawOpen();
        sleep(300);
        delivery.clawToTarget(1850, 3);


        //This code brings two samples to the Obersvation Zone
        Actions.runBlocking(
                new ParallelAction(
                        trajToCollectSamples,
                        delivery.SlideToHeightAction(30),
                        intake.pitchToAngleAction(-250)
                )
        );


        //This code collects the second specimen
//        intake.updateAngle();
        delivery.clawToTarget(800, 2);
        sleep(900);
        Actions.runBlocking(trajToCollectSecondSample);
        delivery.clawClose();
        sleep(400);

        //This code scores the second specimen
        Actions.runBlocking(
                new ParallelAction(
                        trajToScoreSecondSample1,
                        delivery.SlideToHeightAction(2200),
                        intake.pitchToAngleAction(-250)
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        trajToScoreSecondSample2,
                        delivery.SlideToHeightAction(2200)
                )
        );
//        intake.updateAngle();
        delivery.clawToTarget(1650, 3);
        delivery.clawOpen();
        sleep(450);
        sleep(50);

        //Cut for time, add if there is time remaining
//        //This code collects the third specimen
//        Actions.runBlocking(
//                new SequentialAction(
//                        trajToCollectThirdSample1,
//                        delivery.SlideToHeightAction(0)
//                )
//        );
//        delivery.clawToTarget(1000, 2);
//        Actions.runBlocking(trajToCollectThirdSample2);
//        delivery.clawClose();
//        sleep(500);
//
//        //This code scores the third specimen
//        Actions.runBlocking(
//                new ParallelAction(
//                        trajToScoreThirdSample,
//                        delivery.SlideToHeightAction(2000)
//                )
//        );
//        delivery.clawToTarget(1650, 3);
//        delivery.clawOpen();
//        sleep(500);


        //This code parks
        Actions.runBlocking(
                new ParallelAction(
                        trajToPark,
                        delivery.SlideToHeightAction(1500)
                )
        );
        delivery.clawToTarget(0, 5);

        intake.unbrakePitch();

        if (isStopRequested()) return;

    }
}