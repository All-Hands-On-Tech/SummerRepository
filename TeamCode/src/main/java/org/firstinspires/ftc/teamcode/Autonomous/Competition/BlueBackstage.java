package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpmode;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousTrajectories;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.IntakeFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.sql.Time;

@Autonomous(name="Blue Backstage", group="B")
public class BlueBackstage extends RoboMom {

    //logan was here

//    AutonomousTrajectories autoTraj = new AutonomousTrajectories(this);


//    IntakeFunctions intakeFuncts = new IntakeFunctions(this);
    double fx = VisionConstants.fx;
    double fy = VisionConstants.fy;
    double cx = VisionConstants.cx;
    double cy = VisionConstants.cy;

    int RESWIDTH = VisionConstants.RESWIDTH;
    int RESHEIGHT = VisionConstants.RESHEIGHT;

    int TIMEOUT = 5;
    OpenCvCamera webcam;

    SampleMecanumDrive drive;

    CircleDetectionPipeline circleDetectionPipeline = new CircleDetectionPipeline(telemetry, false);

    Pose2d startPose = new Pose2d(-59.5, 14, Math.toRadians(0));

    private static Pose2d endPose = new Pose2d(-34, 38, Math.toRadians(90));

    private String spikePosition = "center";

    @Override
    public void runOpMode() {
        super.runOpMode();
//        autoTraj = new AutonomousTrajectories(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(circleDetectionPipeline);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-40, 21, Math.toRadians(0)), Math.toRadians(-20))
                .setReversed(true)
                .strafeTo(new Vector2d(-53, 21))
                .lineToLinearHeading(new Pose2d(endPose.getX(), endPose.getY(), Math.toRadians(90)))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-33, 15, Math.toRadians(0)), Math.toRadians(0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-40, 9, Math.toRadians(90)), Math.toRadians(-135))
                .strafeTo(new Vector2d(endPose.getX(), endPose.getY()))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-33, 10, Math.toRadians(-90)), Math.toRadians(-90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-40, 15, Math.toRadians(-90)), Math.toRadians(-135))
                .lineToLinearHeading(new Pose2d(endPose.getX(), endPose.getY(), Math.toRadians(90)))
                .build();

        TrajectorySequence leftScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(-32, 42, 90))
                .build();

        TrajectorySequence centerScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(-34, 42, 90))
                .build();

        TrajectorySequence rightScore = drive.trajectorySequenceBuilder(endPose)
                .lineToLinearHeading(new Pose2d(-36, 42, 90))
                .build();


        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(-32, 42, 90))
                .lineToLinearHeading(new Pose2d(32, endPose.getY(), 90))
                .splineToConstantHeading(new Vector2d(-55, 50), Math.toRadians(90))
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(new Pose2d(-34, 42, 90))
                .lineToLinearHeading(new Pose2d(32, endPose.getY(), 90))
                .splineToConstantHeading(new Vector2d(-55, 50), Math.toRadians(90))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(-36, 42, 90))
                .lineToLinearHeading(new Pose2d(32, endPose.getY(), 90))
                .splineToConstantHeading(new Vector2d(-55, 50), Math.toRadians(90))
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
//        circleDetectionPipeline.setState(CircleDetectionPipeline.DetectionState.DETECT);

        spikePosition = MakeDetection(TIMEOUT);
        switch (spikePosition) {
            case "LEFT":
                telemetry.addLine("left");
                telemetry.update();
                drive.followTrajectorySequence(left);

                //CHOPPER PUSHBOT
//                drive.followTrajectorySequence(autoTraj.BlueBackstageLeftTrajectoryChopperPush0);
//                intakeFuncts.OutakeFromIntake(0.1f);
//                sleep(750);
//                intakeFuncts.StopIntakeMotor();
//                drive.followTrajectorySequence(autoTraj.BlueBackstageLeftTrajectoryChopperPush1);


                break;
            case "MID":
                telemetry.addLine("center");
                telemetry.update();
                drive.followTrajectorySequence(center);

                //CHOPPER PUSHBOT
//                drive.followTrajectorySequence(autoTraj.BlueBackstageCenterTrajectoryChopperPush0);
//                intakeFuncts.OutakeFromIntake(0.1f);
//                sleep(750);
//                intakeFuncts.StopIntakeMotor();
//                drive.followTrajectorySequence(autoTraj.BlueBackstageCenterTrajectoryChopperPush1);

                break;
            case "RIGHT":
                telemetry.addLine("right");
                telemetry.update();
                drive.followTrajectorySequence(right);

                //CHOPPER PUSHBOT
//                drive.followTrajectorySequence(autoTraj.BlueBackstageRightTrajectoryChopperPush0);
//                intakeFuncts.OutakeFromIntake(0.1f);
//                sleep(750);
//                intakeFuncts.StopIntakeMotor();
//                drive.followTrajectorySequence(autoTraj.BlueBackstageRightTrajectoryChopperPush1);

                break;
        }

        telemetry.addData("X: ", circleDetectionPipeline.getX());
        telemetry.update();

    }

    public String MakeDetection(int timeoutInSeconds) {
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