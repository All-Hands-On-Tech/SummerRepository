package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousTrajectories;
import org.firstinspires.ftc.teamcode.IntakeFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Red Backstage", group="ARed")
public class RedBackstage extends RoboMom {

    //logan was here

    AutonomousTrajectories autoTraj = new AutonomousTrajectories(this);

    IntakeFunctions intakeFuncts = new IntakeFunctions(this);
    double fx = VisionConstants.fx;
    double fy = VisionConstants.fy;
    double cx = VisionConstants.cx;
    double cy = VisionConstants.cy;

    int RESWIDTH = VisionConstants.RESWIDTH;
    int RESHEIGHT = VisionConstants.RESHEIGHT;
    OpenCvCamera webcam;

    CircleDetectionPipeline circleDetectionPipeline = new CircleDetectionPipeline(telemetry, true);

    Pose2d startPose = new Pose2d(60, 14, Math.toRadians(180));

    String spikePosition = "center";

    private int TIMEOUT = 5;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(circleDetectionPipeline);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(29, 10, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(30, 11), Math.toRadians(180))
                .back(10)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(36, 23), Math.toRadians(180))
                .back(10)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
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

        spikePosition = MakeDetection(TIMEOUT);
        switch (spikePosition) {
            case "LEFT":
                telemetry.addLine("left");
                telemetry.update();
//                drive.followTrajectorySequence(left);
                drive.followTrajectorySequence(autoTraj.RedBackstageLeftTrajectoryChopperPush0);
                intakeFuncts.OutakeFromIntake(0.1f);
                sleep(750);
                intakeFuncts.StopIntakeMotor();
                drive.followTrajectorySequence(autoTraj.RedBackstageLeftTrajectoryChopperPush1);
                break;
            case "MID":
                telemetry.addLine("center");
                telemetry.update();
//                drive.followTrajectorySequence(center);
                drive.followTrajectorySequence(autoTraj.RedBackstageCenterTrajectoryChopperPush0);
                intakeFuncts.OutakeFromIntake(0.1f);
                sleep(750);
                intakeFuncts.StopIntakeMotor();
                drive.followTrajectorySequence(autoTraj.RedBackstageCenterTrajectoryChopperPush1);
                break;
            case "RIGHT":
                telemetry.addLine("right");
                telemetry.update();
//                drive.followTrajectorySequence(right);
                drive.followTrajectorySequence(autoTraj.RedBackstageRightTrajectoryChopperPush0);
                intakeFuncts.OutakeFromIntake(0.1f);
                sleep(750);
                intakeFuncts.StopIntakeMotor();
                drive.followTrajectorySequence(autoTraj.RedBackstageRightTrajectoryChopperPush1);
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
            telemetry.addLine("Defaulted");

            return "MID";
        } else{
            return circleDetectionPipeline.spikePosition;
        }
    }

}