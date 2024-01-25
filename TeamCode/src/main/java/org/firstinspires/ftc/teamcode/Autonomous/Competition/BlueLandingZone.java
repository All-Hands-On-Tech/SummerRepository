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
    Pose2d startPose = new Pose2d(-59.5, -38, Math.toRadians(0));

    private static Pose2d endPose = new Pose2d(-34, 38, Math.toRadians(90));

    String spikePosition = "center";
    int TIMEOUT = 5;

    @Override
    public void runOpMode() {

        super.Initialize(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-19.85, -48.37))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(intakeFunctions.outPower, 0.5))
                .waitSeconds(3)
                .lineTo(new Vector2d(-11.58, -35.48))
                .lineTo(new Vector2d(-12.02, 34.74))
                .splineToLinearHeading(endPose, Math.toRadians(90.00))
                .build();




        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-34.00, -54.33), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-11.51, -39, Math.toRadians(0.00)), Math.toRadians(135.00))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(intakeFunctions.outPower, 1.5d))
                .waitSeconds(5)
                .lineToSplineHeading(new Pose2d(-11.28, -40.68, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-11.28, 36.96))
                .splineToLinearHeading(endPose, Math.toRadians(90.00))
                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-28.11, -37.43, Math.toRadians(-90)), Math.toRadians(90.00))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(intakeFunctions.outPower, 1.5d))
                .waitSeconds(5)
                .lineTo(new Vector2d(-11, -31.03))
                .lineTo(new Vector2d(-11, 35.18))
                .splineToLinearHeading(new Pose2d(endPose.getX(), endPose.getY(), Math.toRadians(90)), Math.toRadians(90.00))
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