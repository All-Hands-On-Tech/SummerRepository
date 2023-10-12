package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="Blue Backstage", group="A")
public class BlueBackstage extends RoboMom {
//logan was here
    double fx = VisionConstants.fx;
    double fy = VisionConstants.fy;
    double cx = VisionConstants.cx;
    double cy = VisionConstants.cy;

    int RESWIDTH = VisionConstants.RESWIDTH;
    int RESHEIGHT = VisionConstants.RESHEIGHT;
    OpenCvCamera webcam;

    CircleDetectionPipeline circleDetectionPipeline = new CircleDetectionPipeline(telemetry);
    Pose2d startPose = new Pose2d(60, 11, Math.toRadians(180));

    String spikePosition = "MID";

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(circleDetectionPipeline);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        //left
        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-35, 11), Math.toRadians(0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-60, 50, Math.toRadians(270)), Math.toRadians(-90))
                .build();

        //mid
        TrajectorySequence midTraj = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-35, 22), Math.toRadians(0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-60, 50, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        //right
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-35, 9, Math.toRadians(-90)), Math.toRadians(-90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-61, 45, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            spikePosition = circleDetectionPipeline.getSpikePosition();

            switch (spikePosition) {
                case "LEFT":
                    drive.followTrajectorySequence(leftTraj);
                    break;
                case "MID":
                    drive.followTrajectorySequence(midTraj);
                    telemetry.addLine("center");
                    break;
                case "RIGHT":
                    drive.followTrajectorySequence(rightTraj);
                    telemetry.addLine("Right");
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