package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.newThing;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="Blue2 Backstage2", group="A")
public class Blue2Backstage2 extends RoboMom {

    //logan was here
    double fx = VisionConstants.fx;
    double fy = VisionConstants.fy;
    double cx = VisionConstants.cx;
    double cy = VisionConstants.cy;

    int RESWIDTH = VisionConstants.RESWIDTH;
    int RESHEIGHT = VisionConstants.RESHEIGHT;
    OpenCvCamera webcam;

    CircleDetectionPipeline circleDetectionPipeline = new CircleDetectionPipeline(telemetry);

    enum State {
        RIGHT,
        CENTER,
        LEFT,
        GOHOME,
        IDLE
    }
    State currentState = State.IDLE;
    Pose2d startPose = new Pose2d(60, 11, Math.toRadians(180));

    String spikePosition = "MID";

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(circleDetectionPipeline);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(30, 11), Math.toRadians(180))
                //.setReversed(true)
                //.splineToLinearHeading(new Pose2d(55, -18, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .build();

        TrajectorySequence goHome = drive.trajectorySequenceBuilder(center.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(40, 11, Math.toRadians(180)))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(55, -18, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        spikePosition = "CENTER"; //circleDetectionPipeline.getSpikePosition();
        switch (spikePosition) {
            case "LEFT":
                currentState = State.LEFT;
                drive.followTrajectorySequenceAsync(left);
                telemetry.addLine("left");
                break;
            case "CENTER":
                    currentState = State.CENTER;
                drive.followTrajectorySequenceAsync(center);
                telemetry.addLine("center");
                break;
            case "RIGHT":
                currentState = State.RIGHT;
                drive.followTrajectorySequenceAsync(right);
                telemetry.addLine("right");
                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case RIGHT:
                    if (!drive.isBusy()) {
                        currentState = State.GOHOME;
                        drive.followTrajectorySequenceAsync(goHome);
                    }
                    break;
                case CENTER:
                    if (!drive.isBusy()) {
                        currentState = State.GOHOME;
                        drive.followTrajectorySequenceAsync(goHome);
                    }
                    break;
                case LEFT:
                    if (!drive.isBusy()) {
                        currentState = State.GOHOME;
                        drive.followTrajectorySequenceAsync(goHome);
                    }
                    break;
                case GOHOME:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    //stops here
                    break;
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addLine(currentState.name());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}