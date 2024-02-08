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

    private static Pose2d endPose = new Pose2d(-34, 34, Math.toRadians(90));

    String spikePosition = "center";
    int TIMEOUT = 5;

    @Override
    public void runOpMode() {

        super.Initialize(this, false, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-45, -50, Math.toRadians(-90)), Math.toRadians(0))
                .strafeRight(5)
                .waitSeconds(4)
                .lineToConstantHeading(new Vector2d(-50, -36))
                .lineToConstantHeading(new Vector2d(-11.5, -36))
                .lineToConstantHeading(new Vector2d(-11.5, 30))
                .splineToLinearHeading(endPose, Math.toRadians(90))
                .build();



        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-24, -47.36), Math.toRadians(0.00))
                .waitSeconds(4)
                .lineToConstantHeading(new Vector2d(-21.67, -55.08))
                .lineToLinearHeading(new Pose2d(-12, -48.84, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(-12, 35.00), Math.toRadians(95.00))
                .splineToLinearHeading(endPose, Math.toRadians(90.00))
                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-35, -40), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(-35, -30.88))
                .lineToConstantHeading(new Vector2d(-35, -45.13))
                .waitSeconds(4)
                .lineToLinearHeading(new Pose2d(-10.84, -46.17, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-10.84, 39.34, Math.toRadians(90)))
                .splineToLinearHeading(endPose, Math.toRadians(90.00))
                .build();



        TrajectorySequence leftScore = drive.trajectorySequenceBuilder(endPose)
                .lineToConstantHeading(new Vector2d(-41, 52))
                .build();

        TrajectorySequence centerScore = drive.trajectorySequenceBuilder(endPose)
                .lineToConstantHeading(new Vector2d(-36.5, 52.25))
                .build();

        TrajectorySequence rightScore = drive.trajectorySequenceBuilder(endPose)
                .lineToConstantHeading(new Vector2d(-28, 52.25))
                .build();


        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(-32, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(-30, endPose.getY() + 7))
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(new Pose2d(-34, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(-30, endPose.getY() + 7))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(-36, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(-30, endPose.getY() + 7))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        sleep(1000);
        spikePosition = MakePropDetection(TIMEOUT);
        switch (spikePosition) {
            case "LEFT":
                targetApriltagID = visionFunctions.BLUE_1_TAG;
                telemetry.addLine("left");
                telemetry.update();
                drive.followTrajectorySequence(left);
                deliveryFunctions.Lift(200);
                drive.followTrajectorySequence(leftScore);
//                MoveToTagForSeconds(targetApriltagID, 5, 1);
                deliveryFunctions.Dump(0);
                drive.followTrajectorySequence(leftPark);
                deliveryFunctions.Retract();
                break;
            case "MID":
                targetApriltagID = visionFunctions.BLUE_2_TAG;
                telemetry.addLine("center");
                telemetry.update();
                drive.followTrajectorySequence(center);
                deliveryFunctions.Lift(250);
                drive.followTrajectorySequence(centerScore);
//                MoveToTagForSeconds(targetApriltagID, 5, 1);
                deliveryFunctions.Dump(0);
                drive.followTrajectorySequence(centerPark);
                deliveryFunctions.Retract();
                break;
            case "RIGHT":
                targetApriltagID = visionFunctions.BLUE_3_TAG;
                telemetry.addLine("right");
                telemetry.update();
                drive.followTrajectorySequence(right);
                deliveryFunctions.Lift(200);
                drive.followTrajectorySequence(rightScore);
//                MoveToTagForSeconds(targetApriltagID, 3, 1);
                deliveryFunctions.Dump(0);
                drive.followTrajectorySequence(rightPark);
                deliveryFunctions.Retract();
                break;
        }

    }


}