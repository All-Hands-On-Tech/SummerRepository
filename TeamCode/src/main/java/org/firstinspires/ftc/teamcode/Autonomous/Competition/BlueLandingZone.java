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

@Autonomous(name="Blue Landing Zone", group="A")
public class BlueLandingZone extends RoboMom {
//logan was here
    double fx = VisionConstants.fx;
    double fy = VisionConstants.fy;
    double cx = VisionConstants.cx;
    double cy = VisionConstants.cy;

    int RESWIDTH = VisionConstants.RESWIDTH;
    int RESHEIGHT = VisionConstants.RESHEIGHT;
    OpenCvCamera webcam;

    CircleDetectionPipeline circleDetectionPipeline = new CircleDetectionPipeline(telemetry);
    Pose2d startPose = new Pose2d(-60, -35, Math.toRadians(0));

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
                .splineToLinearHeading(new Pose2d(-35, -33, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-59, -35, Math.toRadians(90)), Math.toRadians(90))
                .strafeTo(new Vector2d(-59, 12))
                .splineTo(new Vector2d(-35,47), Math.toRadians(90))
                .build();

        //mid
        TrajectorySequence midTraj = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-33, -35), Math.toRadians(0))
                .waitSeconds(5)
                .strafeTo(new Vector2d(-35, -35))
                .setReversed(true)
                .strafeTo(new Vector2d(-35, 47))
                .build();

        //right
        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35, -37, Math.toRadians(-90)))
                .waitSeconds(5)
                .setReversed(true)
                .strafeTo(new Vector2d(-35, 47))
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