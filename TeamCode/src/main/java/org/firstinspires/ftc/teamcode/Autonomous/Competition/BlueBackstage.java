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
    Pose2d startPose = new Pose2d(-64, 14.5, Math.toRadians(0));
    private static Pose2d endPose = new Pose2d(-34, 38, Math.toRadians(90));
    private String spikePosition = "center";

    @Override
    public void runOpMode() {
        super.Initialize(this, false, true);
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-45, 26, Math.toRadians(90)), Math.toRadians(0.00))
                .strafeLeft(5)
                .lineToConstantHeading(new Vector2d(-50, 37.86))
                .lineToSplineHeading(endPose)
                .build();


        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-23.38, 23.31), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-21.38, 35.63))
                .lineToLinearHeading(endPose)
                .build();



        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-35, 20), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-35, 7))
                .lineToConstantHeading(new Vector2d(-35, 30))
                .lineToLinearHeading(endPose)
                .build();


        TrajectorySequence leftScore = drive.trajectorySequenceBuilder(endPose)
                .lineToConstantHeading(new Vector2d(-41, 52))
                .build();

        TrajectorySequence centerScore = drive.trajectorySequenceBuilder(endPose)
                .lineToConstantHeading(new Vector2d(-36.5, 52.25))
                .build();

        TrajectorySequence rightScore = drive.trajectorySequenceBuilder(endPose)
                .lineToConstantHeading(new Vector2d(-30, 52.25))
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

        TrajectorySequence corner = drive.trajectorySequenceBuilder(new Pose2d(-59, 50, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-62, 60))
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
                targetApriltagID = visionFunctions.BLUE_1_TAG;
                telemetry.addLine("left");
                telemetry.update();
                drive.followTrajectorySequence(left);
                deliveryFunctions.Lift(50);
                drive.followTrajectorySequence(leftScore);
                deliveryFunctions.Dump(0);
                drive.followTrajectorySequence(leftPark);
                deliveryFunctions.Retract();
                drive.followTrajectorySequence(corner);
                break;
            case "MID":
                targetApriltagID = visionFunctions.BLUE_2_TAG;
                telemetry.addLine("center");
                telemetry.update();
                drive.followTrajectorySequence(center);
                deliveryFunctions.Lift(50);
                drive.followTrajectorySequence(centerScore);
                deliveryFunctions.Dump(0);
                drive.followTrajectorySequence(centerPark);
                deliveryFunctions.Retract();
                drive.followTrajectorySequence(corner);
                break;
            case "RIGHT":
                targetApriltagID = visionFunctions.BLUE_3_TAG;
                telemetry.addLine("right");
                telemetry.update();
                drive.followTrajectorySequence(right);
                deliveryFunctions.Lift(50);
                drive.followTrajectorySequence(rightScore);
                deliveryFunctions.Dump(0);
                drive.followTrajectorySequence(rightPark);
                deliveryFunctions.Retract();
                drive.followTrajectorySequence(corner);
                break;
        }

//        MoveToTagForSeconds(targetApriltagID, 5);

    }
}