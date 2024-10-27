package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Obs Test", group = "Testing")
public class ObsTest extends LinearOpMode {
    Delivery delivery = null;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(15.2, -62, Math.toRadians(90)));
        delivery = new Delivery(this, false);

        delivery.clawClose();

        Action trajToScoreFirstSample;
        Action trajToCollectSamples;
        Action trajToCollectSecondSample;
        Action trajToScoreSecondSample;
        Action trajToCollectThirdSample1;
        Action trajToCollectThirdSample2;
        Action trajToScoreThirdSample;
        Action trajToPark;

        trajToScoreFirstSample = drive.actionBuilder(drive.pose)
                //Scores pre set specimin
                .strafeTo(new Vector2d(7, -34))
                /*score specimin*/
                .build();

        trajToCollectSamples = drive.actionBuilder(new Pose2d(7, -34, Math.toRadians(90)))
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
                .splineTo(new Vector2d(36, -49), Math.toRadians(-90))
                .build();

        trajToCollectSecondSample = drive.actionBuilder(new Pose2d(55, -58, Math.toRadians(-90)))
                //Scores a second specimin
                /*sleep*/
                .strafeTo(new Vector2d(36, -55))
                .build();

        trajToScoreSecondSample = drive.actionBuilder(new Pose2d(36, -55, Math.toRadians(-90)))
                /*grab specimin*/
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(6, -34, Math.toRadians(90)), Math.toRadians(90))
                /*score specimin*/
                .build();

        trajToCollectThirdSample1 = drive.actionBuilder(new Pose2d(6, -34, Math.toRadians(90)))
                //This scores a third specimin, add if theres time
                .strafeTo(new Vector2d(10, -40))
                .splineToLinearHeading(new Pose2d(36, -49, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        trajToCollectThirdSample2 = drive.actionBuilder(new Pose2d(36, -49, Math.toRadians(-90)))
                .strafeTo(new Vector2d(36, -55))
                .build();
                /*grab specimin*/

        trajToScoreThirdSample = drive.actionBuilder(new Pose2d(36, -55, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(6, -34,Math.toRadians(90)), Math.toRadians(90))
                /*score specimin*/
                .build();

        trajToPark = drive.actionBuilder(new Pose2d(6, -34,Math.toRadians(90)))
                //Returns to observation zone
                .setTangent(Math.toRadians(-90))
                .splineTo(new Vector2d(40, -57), Math.toRadians(-45))
                .build();

        delivery.clawToTarget(4000, 2);
        waitForStart();

        //This code scores preloaded specimin
        Actions.runBlocking(trajToScoreFirstSample);
        delivery.clawToTarget(3100, 3);
        delivery.clawOpen();

        //This code brings two samples to the Obersvation Zone
        Actions.runBlocking(trajToCollectSamples);
        delivery.clawToTarget(2000, 2);

        //This code collects and scores a second specimin
        Actions.runBlocking(trajToCollectSecondSample);
        delivery.clawClose();
        sleep(500);
        delivery.clawToTarget(4000, 2);

        Actions.runBlocking(trajToScoreSecondSample);
        delivery.clawToTarget(3100, 3);
        delivery.clawOpen();

        //This code collects and scores the third specimin
//        Actions.runBlocking(trajToCollectThirdSample1);
//        delivery.clawToTarget(2000, 2);
//
//        Actions.runBlocking(trajToCollectThirdSample2);
//        delivery.clawClose();
//        sleep(500);
//
//        Actions.runBlocking(trajToScoreSecondSample);
//        delivery.clawToTarget(3100, 3);
//        delivery.clawOpen();

        //This code parks
        Actions.runBlocking(trajToPark);
        delivery.clawToTarget(0, 2);

        if (isStopRequested()) return;

    }
}