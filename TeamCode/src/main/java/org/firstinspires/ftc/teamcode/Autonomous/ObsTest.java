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
@Autonomous(name = "Obs", group = "Autonomous")
public class ObsTest extends LinearOpMode {
    Delivery delivery = null;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(15.2, -62, Math.toRadians(90)));
        delivery = new Delivery(this, false);

        Action trajToSubmersible;
        trajToSubmersible = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(7, -34))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(trajToSubmersible);
    }
}