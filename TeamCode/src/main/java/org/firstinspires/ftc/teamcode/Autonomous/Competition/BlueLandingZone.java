package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpmode;
import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.IntakeFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Blue Landing Zone", group="B")
public class BlueLandingZone extends AutonomousOpmode {

    //logan was here
    Pose2d startPose = new Pose2d(-63, -38, Math.toRadians(0));

    private static Pose2d endPose = new Pose2d(-34, 38, Math.toRadians(90));

    String spikePosition = "center";
    int TIMEOUT = 5;

    @Override
    public void runOpMode() {

        super.Initialize(this, false, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-46.02, -56.26), Math.toRadians(-23.65))
                .lineToConstantHeading(new Vector2d(-24.49, -56.26))
                .splineToLinearHeading(new Pose2d(-18.56, -41.86, Math.toRadians(90.00)), Math.toRadians(90.90))
                .lineToConstantHeading(new Vector2d(-10.54, -41.86))
                .splineToConstantHeading(new Vector2d(-11.43, 39.04), Math.toRadians(95.00))
                .splineToLinearHeading(endPose, Math.toRadians(90))
                .build();





        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-21.97, -47.36), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(-21.67, -55.08))
                .lineToLinearHeading(new Pose2d(-10.69, -48.84, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(-10.69, 38.00), Math.toRadians(95.00))
                .splineToLinearHeading(endPose, Math.toRadians(90.00))
                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-33.25, -30.88), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(-33.55, -45.13))
                .lineToLinearHeading(new Pose2d(-9.95, -46.17, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-10.84, 39.34), Math.toRadians(95.00))
                .splineToLinearHeading(endPose, Math.toRadians(90.00))
                .build();



        TrajectorySequence leftScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(-40.25, 52, Math.toRadians(90)))
                .build();

        TrajectorySequence centerScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(-33, 52, Math.toRadians(90)))
                .build();

        TrajectorySequence rightScore = drive.trajectorySequenceBuilder(endPose)
                .splineToLinearHeading(new Pose2d(-27, 52, Math.toRadians(90)), Math.toRadians(90))
                .build();


        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(-32, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(-30, endPose.getY() + 5))
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(new Pose2d(-34, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(-30, endPose.getY() + 5))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(-36, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(-30, endPose.getY() + 5))
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
                drive.followTrajectorySequence(leftScore);
                deliveryFunctions.Score(100);
                drive.followTrajectorySequence(leftPark);
                deliveryFunctions.Retract();
                break;
            case "MID":
                telemetry.addLine("center");
                telemetry.update();
                drive.followTrajectorySequence(center);
                drive.followTrajectorySequence(centerScore);
                deliveryFunctions.Score(100);
                drive.followTrajectorySequence(centerPark);
                deliveryFunctions.Retract();
                break;
            case "RIGHT":
                telemetry.addLine("right");
                telemetry.update();
                drive.followTrajectorySequence(right);
                drive.followTrajectorySequence(rightScore);
                deliveryFunctions.Score(100);
                drive.followTrajectorySequence(rightPark);
                deliveryFunctions.Retract();
                break;
        }

    }


}