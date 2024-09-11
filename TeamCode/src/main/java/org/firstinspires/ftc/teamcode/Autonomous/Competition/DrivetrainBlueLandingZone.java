package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="DrivetrainBlue Landing Zone", group="M")
public class DrivetrainBlueLandingZone extends RoboMom {

    //logan was here

//    AutonomousTrajectories autoTraj = new AutonomousTrajectories(this);

//    IntakeFunctions intakeFuncts = new IntakeFunctions(this);

    double fx = VisionConstants.fx;
    double fy = VisionConstants.fy;
    double cx = VisionConstants.cx;
    double cy = VisionConstants.cy;

    int RESWIDTH = VisionConstants.RESWIDTH;
    int RESHEIGHT = VisionConstants.RESHEIGHT;
    OpenCvCamera webcam;

    CircleDetectionPipeline circleDetectionPipeline = new CircleDetectionPipeline(telemetry, false);

    DeliveryFunctions deliveryFunctions;

    Pose2d startPose = new Pose2d(-59.5, -38, Math.toRadians(0));

    private static Pose2d endPose = new Pose2d(-34, 38, Math.toRadians(90));

    String spikePosition = "center";
    int TIMEOUT = 5;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(circleDetectionPipeline);

//        deliveryFunctions = new DeliveryFunctions(this, true);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-30, -37, Math.toRadians(-90)))
                .waitSeconds(1)
                .back(2)
                .lineToConstantHeading(new Vector2d(-11, -37))
                .turn(Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-11, 30))
                .strafeTo(new Vector2d(endPose.getX(), endPose.getY()))
//                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-33, -35), Math.toRadians(0))
                .waitSeconds(5)
                .strafeTo(new Vector2d(-36, -55))
                .splineToLinearHeading(new Pose2d(-12, -35, Math.toRadians(90)), Math.toRadians(90))
                .strafeTo(new Vector2d(-12, 30))
                .strafeTo(new Vector2d(endPose.getX(), endPose.getY()))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-35, -33, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, -35, Math.toRadians(90)), Math.toRadians(90))
                .strafeTo(new Vector2d(-12, 30))
                .strafeTo(new Vector2d(endPose.getX(), endPose.getY()))
                .build();

        TrajectorySequence leftScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(-40, 52, Math.toRadians(90)))
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

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            public void onOpened()
            {
                webcam.startStreaming(RESWIDTH, RESHEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode){

            }
        });

        waitForStart();
        if (isStopRequested()) return;

        sleep(1000);
        spikePosition = MakeDetection(TIMEOUT);
        switch (spikePosition) {
            case "LEFT":
                telemetry.addLine("left");
                telemetry.update();
                drive.followTrajectorySequence(left);
                drive.followTrajectorySequence(leftScore);
//                deliveryFunctions.Score();
                drive.followTrajectorySequence(leftPark);
//                deliveryFunctions.Retract();
                break;
            case "MID":
                telemetry.addLine("center");
                telemetry.update();
                drive.followTrajectorySequence(center);
                drive.followTrajectorySequence(centerScore);
//                deliveryFunctions.Score();
                drive.followTrajectorySequence(centerPark);
//                deliveryFunctions.Retract();
                break;
            case "RIGHT":
                telemetry.addLine("right");
                telemetry.update();
                drive.followTrajectorySequence(right);
                drive.followTrajectorySequence(rightScore);
//                deliveryFunctions.Score();
                drive.followTrajectorySequence(rightPark);
//                deliveryFunctions.Retract();
                break;
        }

    }

    public String MakeDetection(int timeoutInSeconds) {
        int tries = 0;
        while (opModeIsActive() && !circleDetectionPipeline.isDetected() && tries < timeoutInSeconds * 10) {
            sleep(50);
            tries++;
            telemetry.addData("Detection tries:", tries);
        }
        if (!circleDetectionPipeline.isDetected()){
            return "MID";
        } else{
            return circleDetectionPipeline.spikePosition;
        }
    }


}