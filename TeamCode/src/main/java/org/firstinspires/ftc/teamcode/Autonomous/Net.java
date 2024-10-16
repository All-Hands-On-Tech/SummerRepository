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
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Net", group = "Autonomous")
public class Net extends LinearOpMode {
    Delivery delivery = null;


    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-15.2, -62, Math.toRadians(90)));
        delivery = new Delivery(this, false);

        Action trajToSubmersable;
        Action trajToScoreSamplesInNet;
        Action trajToPark;

        //FIXME:    SPLIT trajectoryAction1 INTO MULTIPLE ACTIONS SO DELIVERY ACTIONS CAN BE IMPLEMENTED IN PARALLEL / SEQUENTIALLY

        trajToSubmersable = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-7, -34))
                .build();

        trajToScoreSamplesInNet = drive.actionBuilder(new Pose2d(-10, -34, Math.toRadians(90)))
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
                .build();

        trajToPark = drive.actionBuilder(new Pose2d(-62, -55, Math.toRadians(0)))
                .strafeTo(new Vector2d(-40, -55))
                .strafeTo(new Vector2d(-40, -10))
                .strafeTo(new Vector2d(-24, -10))
                .build();

        delivery.clawClose();
        waitForStart();

        if (isStopRequested()) return;

        delivery.setSlidesTargetPosition(4000);
        while (Math.abs(delivery.getMotorTargetPosition() - delivery.getMotorPosition()) > 20) {
            delivery.PControlPower(2);
        }
        sleep(500);

        Actions.runBlocking(trajToSubmersable);

        delivery.setSlidesTargetPosition(3100);
        while (Math.abs(delivery.getMotorTargetPosition() - delivery.getMotorPosition()) > 20) {
            delivery.PControlPower(3);
        }
        delivery.clawOpen();
        sleep(1000);

        Actions.runBlocking(trajToScoreSamplesInNet);


        delivery.setSlidesTargetPosition(0);
        while (Math.abs(delivery.getMotorTargetPosition() - delivery.getMotorPosition()) > 20) {
            delivery.PControlPower(3);
        }

        Actions.runBlocking(trajToPark);
    }
}