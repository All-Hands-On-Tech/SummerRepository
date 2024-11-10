package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

        Action goingForward;
        Action goingBackward;

        goingForward = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .strafeTo(new Vector2d(0, 24))
                .build();
        goingBackward = drive.actionBuilder(new Pose2d(0, 24, Math.toRadians(90)))
                .strafeTo(new Vector2d(0, 0))
                .build();

        waitForStart();

        Actions.runBlocking(goingForward);
        Actions.runBlocking(goingBackward);
        Actions.runBlocking(goingForward);
        Actions.runBlocking(goingBackward);

        if (isStopRequested()) return;

    }
}