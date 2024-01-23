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
    double fx = VisionConstants.fx;
    double fy = VisionConstants.fy;
    double cx = VisionConstants.cx;
    double cy = VisionConstants.cy;

    int RESWIDTH = VisionConstants.RESWIDTH;
    int RESHEIGHT = VisionConstants.RESHEIGHT;
    OpenCvCamera webcam;

    CircleDetectionPipeline circleDetectionPipeline = new CircleDetectionPipeline(telemetry, true);

    DeliveryFunctions deliveryFunctions;

    Pose2d startPose = new Pose2d(59.5, 14, Math.toRadians(180));

    private static Pose2d endPose = new Pose2d(34, 38, Math.toRadians(90));

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
//                .splineToLinearHeading(new Pose2d(32, 10, Math.toRadians(-90)), Math.toRadians(-90))
//                .setReversed(true)
//                .strafeTo(new Vector2d(53, 21))
//                .setReversed(false)
//                .lineToLinearHeading(new Pose2d(endPose.getX(), endPose.getY(), Math.toRadians(90)))
//                .build();
//
//        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(34.5, 12, Math.toRadians(180)), Math.toRadians(180))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(43, 7, Math.toRadians(90)), Math.toRadians(90))
//                .setReversed(false)
//                .strafeTo(new Vector2d(endPose.getX(), endPose.getY()))
//                .build();
//
//        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(39, 23, Math.toRadians(180)), Math.toRadians(90))
//                .back(2)
//                .setReversed(true)
//                .strafeTo(new Vector2d(40, 30))
//                .setReversed(false)
//                .lineToLinearHeading(new Pose2d(endPose.getX(), endPose.getY(), Math.toRadians(90)))
//                .build();

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