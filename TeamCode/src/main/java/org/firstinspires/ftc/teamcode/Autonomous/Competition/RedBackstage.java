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

        super.Initialize(this);
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(startPose.getX(), startPose.getY(), Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-32.51, 32.36, Math.toRadians(90)), Math.toRadians(90.00))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(intakeFunctions.outPower, 0.5))
                .splineToLinearHeading(endPose, Math.toRadians(90))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(new Pose2d(startPose.getX(), startPose.getY(), Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-24.49, 27.46, Math.toRadians(90)), Math.toRadians(-90.00))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(intakeFunctions.outPower, 0.5))
                .splineToLinearHeading(endPose, Math.toRadians(90))
                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(33.55, 9.95, Math.toRadians(90)), Math.toRadians(90.00))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(intakeFunctions.outPower, 0.5))
                .splineToLinearHeading(endPose, Math.toRadians(90))
                .build();


        TrajectorySequence leftScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(23, 52, Math.toRadians(90)))
                .build();

        TrajectorySequence centerScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(38, 52, Math.toRadians(90)))
                .build();

        TrajectorySequence rightScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(42.5, 52, Math.toRadians(90)))
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(32, 42, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(32, endPose.getY(), Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(55, 50), Math.toRadians(90))
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(new Pose2d(34, 42, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(32, endPose.getY(), Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(55, 50), Math.toRadians(90))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(36, 42, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(32, endPose.getY(), Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(58, 50), Math.toRadians(90))
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
                deliveryFunctions.Score(0);
                drive.followTrajectorySequence(leftPark);
                deliveryFunctions.Retract();
                break;
            case "MID":
                telemetry.addLine("center");
                telemetry.update();
                drive.followTrajectorySequence(center);
                drive.followTrajectorySequence(centerScore);
                deliveryFunctions.Score(0);
                drive.followTrajectorySequence(centerPark);
                deliveryFunctions.Retract();
                break;
            case "RIGHT":
                telemetry.addLine("right");
                telemetry.update();
                drive.followTrajectorySequence(right);
                drive.followTrajectorySequence(rightScore);
                deliveryFunctions.Score(0);
                drive.followTrajectorySequence(rightPark);
                deliveryFunctions.Retract();
                break;
        }

    }

}