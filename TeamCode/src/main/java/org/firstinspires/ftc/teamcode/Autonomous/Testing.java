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
@Autonomous(name = "Testing", group = "Testing")
public class Testing extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));

        Action traj1;
        Action traj2;

        traj1 = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .strafeTo(new Vector2d(0, 24))
                .build();
        traj2 = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .strafeTo(new Vector2d(0, 48))
                .build();

        waitForStart();

        //This code scores preloaded specimin
        Actions.runBlocking(traj1);
        sleep(2000);
        Actions.runBlocking(traj2);

        if (isStopRequested()) return;

    }
}