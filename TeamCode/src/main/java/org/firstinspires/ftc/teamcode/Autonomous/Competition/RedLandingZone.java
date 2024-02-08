package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpmode;
import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Red Landing Zone", group="ARed")
public class RedLandingZone extends AutonomousOpmode {

    //logan was here

    SampleMecanumDrive drive;
    Pose2d startPose = new Pose2d(63, -38, Math.toRadians(180));
    private static Pose2d endPose = new Pose2d(34, 38, Math.toRadians(90));
    private String spikePosition = "center";

    @Override
    public void runOpMode() {
        super.Initialize(this, true, false);
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(45, -50, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeLeft(5)
                .waitSeconds(3)
                .lineToConstantHeading(new Vector2d(50, -36))
                .lineToConstantHeading(new Vector2d(11.5, -36))
                .lineToConstantHeading(new Vector2d(11.5, 30))
                .splineToLinearHeading(endPose, Math.toRadians(90))
                .build();



        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(23, -47.36), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(21.67, -55.08))
                .lineToLinearHeading(new Pose2d(11, -48.84, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(11.5, 35.00), Math.toRadians(85.00))
                .splineToLinearHeading(endPose, Math.toRadians(90.00))
                .build();



        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(34, startPose.getY()), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(34, -30.88))
                .lineToConstantHeading(new Vector2d(34, -45.13))
                .lineToLinearHeading(new Pose2d(10.39, -44.68, Math.toRadians(90.00)))
                .lineTo(new Vector2d(10.84, 39.34))
                .splineToLinearHeading(endPose, Math.toRadians(90.00))
                .build();



        TrajectorySequence leftScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(44, 51, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(28.5, 51))
                .build();

        TrajectorySequence centerScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(29.5, 51.25, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(35.1, 51))
                .build();

        TrajectorySequence rightScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(29, 51.4, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(42, 51.4))
                .build();



        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(32, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(endPose.getX(), endPose.getY() + 5))
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(new Pose2d(34, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(endPose.getX(), endPose.getY() + 5))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(36, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(endPose.getX(), endPose.getY() + 5))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        sleep(1000);

        spikePosition = MakePropDetection(TIMEOUT);
        switch (spikePosition) {
            case "LEFT":
                telemetry.addLine("left");
                telemetry.update();
                drive.followTrajectorySequence(left);
                deliveryFunctions.Lift(50);
                drive.followTrajectorySequence(leftScore);
                deliveryFunctions.Dump(0);
                drive.followTrajectorySequence(leftPark);
                deliveryFunctions.Retract();
                break;
            case "MID":
                telemetry.addLine("center");
                telemetry.update();
                drive.followTrajectorySequence(center);
                deliveryFunctions.Lift(50);
                drive.followTrajectorySequence(centerScore);
                deliveryFunctions.Dump(0);
                drive.followTrajectorySequence(centerPark);
                deliveryFunctions.Retract();
                break;
            case "RIGHT":
                telemetry.addLine("right");
                telemetry.update();
                drive.followTrajectorySequence(right);
                deliveryFunctions.Lift(50);
                drive.followTrajectorySequence(rightScore);
                deliveryFunctions.Dump(0);
                drive.followTrajectorySequence(rightPark);
                deliveryFunctions.Retract();
                break;
        }

    }

}