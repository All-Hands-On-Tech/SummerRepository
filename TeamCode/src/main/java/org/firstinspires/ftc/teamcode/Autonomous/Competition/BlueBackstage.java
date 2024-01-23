package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpmode;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Blue Backstage", group="B")
public class BlueBackstage extends AutonomousOpmode {

    //logan was here
    SampleMecanumDrive drive;
    Pose2d startPose = new Pose2d(-59.5, 14, Math.toRadians(0));
    private static Pose2d endPose = new Pose2d(-34, 38, Math.toRadians(90));
    private String spikePosition = "center";

    @Override
    public void runOpMode() {
        super.Initialize(this);
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-32.51, 32.36), Math.toRadians(90.00))
//                .addTemporalMarker(()->intakeFunctions.OutakeFromIntakeForTime(0.3f, 0.5))
                .splineTo(new Vector2d(endPose.getX(), endPose.getY()), Math.toRadians(90))
                .build();


        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-24.64, 27.46), Math.toRadians(-45.00))
//                .addTemporalMarker(()->intakeFunctions.OutakeFromIntakeForTime(0.3f, 0.5))
                .splineTo(new Vector2d(endPose.getX(), endPose.getY()), Math.toRadians(90.00))
                .build();


        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-33.55, 9.95), Math.toRadians(90.00))
//                .addTemporalMarker(()->intakeFunctions.OutakeFromIntakeForTime(0.3f, 0.5))
                .splineTo(new Vector2d(endPose.getX(), endPose.getY()), Math.toRadians(90.00))
                .build();

        TrajectorySequence leftScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(-41, 52, Math.toRadians(90)))
                .build();

        TrajectorySequence centerScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(-33.5, 52, Math.toRadians(90)))
                .build();

        TrajectorySequence rightScore = drive.trajectorySequenceBuilder(endPose)
                .splineToLinearHeading(new Pose2d(-27.5, 52, Math.toRadians(90)), Math.toRadians(90))
                .build();


        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(-32, 42, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-32, endPose.getY() + 5, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-59, 50), Math.toRadians(90))
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(new Pose2d(-34, 42, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-32, endPose.getY(), Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-59, 50), Math.toRadians(90))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(-36, 42, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-32, endPose.getY(), Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-59, 50), Math.toRadians(90))
                .build();

//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
//            public void onOpened()
//            {
//                webcam.startStreaming(RESWIDTH, RESHEIGHT, OpenCvCameraRotation.UPRIGHT);
//            }
//            public void onError(int errorCode){
//
//            }
//        });

        waitForStart();
        if (isStopRequested()) return;

        sleep(1000);
//        circleDetectionPipeline.setState(CircleDetectionPipeline.DetectionState.DETECT);

        spikePosition = MakePropDetection(TIMEOUT);
        switch (spikePosition) {
            case "LEFT":
                telemetry.addLine("left");
                telemetry.update();
                drive.followTrajectorySequence(left);
                drive.followTrajectorySequence(leftScore);
                deliveryFunctions.Score();
                drive.followTrajectorySequence(leftPark);
                deliveryFunctions.Retract();
                break;
            case "MID":
                telemetry.addLine("center");
                telemetry.update();
                drive.followTrajectorySequence(center);
                drive.followTrajectorySequence(centerScore);
                deliveryFunctions.Score();
                drive.followTrajectorySequence(centerPark);
                deliveryFunctions.Retract();
                break;
            case "RIGHT":
                telemetry.addLine("right");
                telemetry.update();
                drive.followTrajectorySequence(right);
                drive.followTrajectorySequence(rightScore);
                deliveryFunctions.Score();
                drive.followTrajectorySequence(rightPark);
                deliveryFunctions.Retract();
                break;
        }

        MoveToTagForSeconds(0, 1);

    }
}