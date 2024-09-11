package org.firstinspires.ftc.teamcode.Autonomous.Other;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoboMom;
@Disabled
public class NewNewThing extends RoboMom{

    enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        ROTATION,
        IDLE
    }

    NewNewThing.State currentState = NewNewThing.State.IDLE;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0, 24))
                .addDisplacementMarker(0.5, 0, () -> {
                    telemetry.addLine("halfway through traj1");
                    telemetry.update();
                })
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToSplineHeading(new Pose2d(0, -24, Math.toRadians(0)), Math.toRadians(180))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        currentState = NewNewThing.State.TRAJECTORY_1;
        drive.followTrajectoryAsync(traj1);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case TRAJECTORY_1:
                    if (!drive.isBusy()) {
                        currentState = NewNewThing.State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(traj2);
                    }
                    break;
                case TRAJECTORY_2:
                    if (!drive.isBusy()) {
                        currentState = NewNewThing.State.IDLE;
                    }
                    break;
                case IDLE:
                    //stops here
                    break;
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

}
