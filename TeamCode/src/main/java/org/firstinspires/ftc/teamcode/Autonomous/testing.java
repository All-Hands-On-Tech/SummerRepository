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
import org.firstinspires.ftc.teamcode.RoadRunner.SparkFunOTOSDrive;

@Config
@Autonomous(name = "Testing", group = "Testing")
public class testing extends LinearOpMode {

    @Override
    public void runOpMode() {
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        Action traj1;
        Action traj2;

        traj1 = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                //Scores pre set specimin
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(180)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-0, 0, Math.toRadians(90)), Math.toRadians(-90))
                /*score specimin*/
                .build();

        traj2 = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .strafeTo(new Vector2d(-10, 30))
                .strafeTo(new Vector2d(-30, 10))
                .strafeTo(new Vector2d(-0, 0))
                .turn(Math.PI*10)
                .build();

        waitForStart();

        //This code scores preloaded specimen
        Actions.runBlocking(traj2);
//

        if (isStopRequested()) return;

    }
}