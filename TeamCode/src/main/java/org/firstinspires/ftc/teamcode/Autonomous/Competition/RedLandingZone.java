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

@Autonomous(name="Red Landing Zone", group="ARed")
public class RedLandingZone extends AutonomousOpmode {

    //logan was here

    private static Pose2d endPose = new Pose2d(34, 38, Math.toRadians(90));

//    AutonomousTrajectories autoTraj = new AutonomousTrajectories(this);

//    IntakeFunctions intakeFuncts = new IntakeFunctions(this);
    DeliveryFunctions deliveryFunctions;
    double fx = VisionConstants.fx;
    double fy = VisionConstants.fy;
    double cx = VisionConstants.cx;
    double cy = VisionConstants.cy;

    int RESWIDTH = VisionConstants.RESWIDTH;
    int RESHEIGHT = VisionConstants.RESHEIGHT;
    OpenCvCamera webcam;

    CircleDetectionPipeline circleDetectionPipeline = new CircleDetectionPipeline(telemetry, true);

    Pose2d startPose = new Pose2d(59.5, -37, Math.toRadians(180));

    String spikePosition = "center";

    private int TIMEOUT = 5;

    @Override
    public void runOpMode() {


        deliveryFunctions = new DeliveryFunctions(this, true);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(circleDetectionPipeline);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

//        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(30, -37, Math.toRadians(-90)))
//                .waitSeconds(5)
//                .back(2)
//                .lineToConstantHeading(new Vector2d(9, -37))
//                .lineToConstantHeading(new Vector2d(9, 30))
//                .strafeTo(new Vector2d(endPose.getX(), endPose.getY()))
//                .turn(Math.toRadians(180))
//                .build();
//
//        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
//                .splineTo(new Vector2d(33, -35), Math.toRadians(180))
//                .waitSeconds(5)
//                .strafeTo(new Vector2d(39, -48))
//                .splineToLinearHeading(new Pose2d(12, -35, Math.toRadians(90)), Math.toRadians(90))
//                .strafeTo(new Vector2d(12, 30))
//                .strafeTo(new Vector2d(endPose.getX(), endPose.getY()))
//                .build();
//
//        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(35, -33, Math.toRadians(90)), Math.toRadians(90))
//                .waitSeconds(5)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(12, -35, Math.toRadians(90)), Math.toRadians(90))
//                .strafeTo(new Vector2d(12, 30))
//                .strafeTo(new Vector2d(endPose.getX(), endPose.getY()))
//                .build();
//
//        TrajectorySequence leftScore = drive.trajectorySequenceBuilder(endPose)
//                .lineToLinearHeading(new Pose2d(35, 52, Math.toRadians(90)))
//                .build();
//
//        TrajectorySequence centerScore = drive.trajectorySequenceBuilder(endPose)
//                .lineToLinearHeading(new Pose2d(40, 52, Math.toRadians(90)))
//                .build();
//
//        TrajectorySequence rightScore = drive.trajectorySequenceBuilder(endPose)
//                .lineToLinearHeading(new Pose2d(42.5, 52, Math.toRadians(90)))
//                .build();
//
//
//
//        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(32, 42, Math.toRadians(90)))
//                .back(2)
//                .strafeTo(new Vector2d(endPose.getX(), endPose.getY() + 5))
//                .build();
//
//        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(new Pose2d(34, 42, Math.toRadians(90)))
//                .back(2)
//                .strafeTo(new Vector2d(endPose.getX(), endPose.getY() + 5))
//                .build();
//
//        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(36, 42, Math.toRadians(90)))
//                .back(2)
//                .strafeTo(new Vector2d(endPose.getX(), endPose.getY() + 5))
//                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(33.85, -36.37, Math.toRadians(90.00)))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(0.1f, 0.5))
                .waitSeconds(5)
                .lineTo(new Vector2d(11.58, -35.48))
                .lineTo(new Vector2d(12.02, 34.74))
                .splineTo(new Vector2d(endPose.getX(), endPose.getY()), Math.toRadians(90.00))
                .build();


        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(34.00, -54.33), Math.toRadians(180.00))
                .splineToSplineHeading(new Pose2d(13.51, -40.38, Math.toRadians(180.00)), Math.toRadians(45.00))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(0.1f, 0.5))
                .waitSeconds(5)
                .lineToSplineHeading(new Pose2d(11.28, -40.68, Math.toRadians(90.00)))
                .lineTo(new Vector2d(11.28, 36.96))
                .splineTo(new Vector2d(endPose.getX(), endPose.getY()), Math.toRadians(90.00))
                .build();


        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(18.11, -30.43), Math.toRadians(90.00))
                .addDisplacementMarker(()->intakeFunctions.OutakeFromIntakeForTime(0.1f, 0.5))
                .lineTo(new Vector2d(12.17, -31.03))
                .lineTo(new Vector2d(12.17, 35.18))
                .splineTo(new Vector2d(36.07, 46.47), Math.toRadians(90.00))
                .build();


        TrajectorySequence leftScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(35, 52, Math.toRadians(90)))
                .build();

        TrajectorySequence centerScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(40, 52, Math.toRadians(90)))
                .build();

        TrajectorySequence rightScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(42.5, 52, Math.toRadians(90)))
                .build();



        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(32, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(endPose.getX(), endPose.getY() + 5))
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(new Pose2d(34, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(endPose.getX(), endPose.getY() + 5))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(36, 42, Math.toRadians(90)))
                .back(2)
                .strafeTo(new Vector2d(endPose.getX(), endPose.getY() + 5))
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

    }

    public String MakePropDetection(int timeoutInSeconds) {
        int tries = 0;
        while (opModeIsActive() && !circleDetectionPipeline.isDetected() && tries < timeoutInSeconds * 10) {
            sleep(50);
            tries++;
            telemetry.addData("Detection tries:", tries);
        }
        if (!circleDetectionPipeline.isDetected()){
            telemetry.addLine("Defaulted");

            return "MID";
        } else{
            return circleDetectionPipeline.spikePosition;
        }
    }
}