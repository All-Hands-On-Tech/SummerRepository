package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Testing", group = "Testing")
public class Testing extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, -48, Math.toRadians(90)));
        Action traj1;
        Action traj2;

        traj1 = drive.actionBuilder(new Pose2d(0, -48, Math.toRadians(90)))
                .splineTo(new Vector2d(-12, -24), Math.toRadians(90))
                .splineTo(new Vector2d(0, 0), Math.toRadians(30))
                .splineTo(new Vector2d(12, 24), Math.toRadians(90))
                .splineTo(new Vector2d(0, 48), Math.toRadians(150))
                .build();

        traj2 = drive.actionBuilder(new Pose2d(0, 48, Math.toRadians(150)))
                .turnTo(Math.toRadians(-90))
                .setTangent(Math.toRadians(210))
                .splineToLinearHeading(new Pose2d(-12, 24, Math.toRadians(-45)), Math.toRadians(-90))
                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(-30))
                .setTangent(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(12, -24, Math.toRadians(45)), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(0, -48, Math.toRadians(90)), Math.toRadians(-90))
                .build();

        waitForStart();

        Actions.runBlocking(traj1);
        Actions.runBlocking(traj2);

        if (isStopRequested()) return;

    }
}