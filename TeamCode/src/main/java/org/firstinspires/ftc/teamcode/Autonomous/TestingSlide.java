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
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Testing Slide", group = "Testing")
public class TestingSlide extends LinearOpMode {
    Delivery delivery = null;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        delivery = new Delivery(this, false);

        Action traj;

        traj = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(0, 40))
                .build();

        waitForStart();

        //This code scores preloaded specimin
        Actions.runBlocking(
                new ParallelAction(
                        traj,
                        delivery.SlideToHeightAction(3000)
                )
        );

        if (isStopRequested()) return;

    }
}