package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Obs", group = "Autonomous")
public class Obs extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(25, -62, Math.toRadians(90)));

        Action trajectoryAction1;
        Action trajectoryAction2;

        //FIXME:    SPLIT trajectoryAction1 INTO MULTIPLE ACTIONS SO DELIVERY ACTIONS CAN BE IMPLEMENTED IN PARALLEL / SEQUENTIALLY

        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(10, -34))
                //score specimen
                .strafeTo(new Vector2d(20, -45))
                .setTangent(-45)
                .splineToLinearHeading(new Pose2d(45, -14, Math.toRadians(90)), Math.toRadians(-70))
                .strafeTo(new Vector2d(45, -59))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(55.00, -11, Math.toRadians(90)), Math.toRadians(-90))
                .strafeTo(new Vector2d(55, -60))
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(62, -24.10, Math.toRadians(180)), Math.toRadians(0))
                .strafeTo(new Vector2d(62, -62))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction1
                )
        );
    }
}