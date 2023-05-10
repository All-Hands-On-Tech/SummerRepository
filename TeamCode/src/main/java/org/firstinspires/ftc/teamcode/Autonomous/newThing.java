package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

public class newThing extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(12)
                .forward(12)
                .build();

        Trajectory coolT = drive.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(-24,-24,Math.toRadians(180)),Math.toRadians(90))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
        drive.followTrajectory(myTrajectory);
        drive.followTrajectory();
    }
}
