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

@Autonomous(name="Red Backstage", group="ARed")
public class RedBackstage extends AutonomousOpmode {

    //logan was here

//    AutonomousTrajectories autoTraj = new AutonomousTrajectories(this);

//    IntakeFunctions intakeFuncts = new IntakeFunctions(this);

    SampleMecanumDrive drive;

    Pose2d startPose = new Pose2d(59.5, 14, Math.toRadians(180));

    private static Pose2d endPose = new Pose2d(34, 38, Math.toRadians(90));

    private String spikePosition = "center";

    @Override
    public void runOpMode() {

        super.Initialize(this, true, true);
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(45.51, 22, Math.toRadians(0)), Math.toRadians(0.00))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(intakeFunctions.outPower, 0.5))
                .splineToLinearHeading(endPose, Math.toRadians(90))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(24, 29, Math.toRadians(90)), Math.toRadians(-90.00))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(intakeFunctions.outPower, 0.5))
                .splineToLinearHeading(endPose, Math.toRadians(90))
                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(41, 17, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(41, 9, Math.toRadians(45)), Math.toRadians(45.00))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(intakeFunctions.outPower, 0.5))
                .splineToLinearHeading(endPose, Math.toRadians(90))
                .build();


        TrajectorySequence leftScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(27, 51.5, Math.toRadians(90)))
                .build();

        TrajectorySequence centerScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(35.25, 51.5, Math.toRadians(90)))
                .build();

        TrajectorySequence rightScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(41, 50.5, Math.toRadians(90)))
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(32, 42, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(35, endPose.getY(), Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(58, 50), Math.toRadians(90))
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(new Pose2d(34, 42, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(35, endPose.getY(), Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(58, 50), Math.toRadians(90))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(36, 42, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(35, endPose.getY()+4, Math.toRadians(90)))
                .lineTo(new Vector2d(57, endPose.getY()+4))
                .splineToConstantHeading(new Vector2d(57, 50), Math.toRadians(90))
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
                deliveryFunctions.Score(-250);
                drive.followTrajectorySequence(leftPark);
                deliveryFunctions.Retract();
                break;
            case "MID":
                telemetry.addLine("center");
                telemetry.update();
                drive.followTrajectorySequence(center);
                drive.followTrajectorySequence(centerScore);
                deliveryFunctions.Score(-250);
                drive.followTrajectorySequence(centerPark);
                deliveryFunctions.Retract();
                break;
            case "RIGHT":
                telemetry.addLine("right");
                telemetry.update();
                drive.followTrajectorySequence(right);
                drive.followTrajectorySequence(rightScore);
                deliveryFunctions.Score(-250);
                drive.followTrajectorySequence(rightPark);
                deliveryFunctions.Retract();
                break;
        }

    }

}