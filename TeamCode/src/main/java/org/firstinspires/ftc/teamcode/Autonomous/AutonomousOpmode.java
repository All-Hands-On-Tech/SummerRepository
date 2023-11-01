package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
public class AutonomousOpmode extends LinearOpMode {

    private OpenCvWebcam webcam;
    protected CircleDetectionPipeline circleDetection;
    protected SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    protected Pose2d startPose;
    protected boolean isRed;
    protected boolean isBackstage;
    protected String spikePosition;


    private void Initialize() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        circleDetection = new CircleDetectionPipeline(isRed);
        webcam.setPipeline(circleDetection);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        if(isRed){
            if(isBackstage){
                startPose = RED_BACKSTAGE_START_POSE;
            } else{
                startPose = RED_LANDING_ZONE_START_POSE;
            }
        } else{
            if(isBackstage){
                startPose = BLUE_BACKSTAGE_START_POSE;
            } else{
                startPose = BLUE_LANDING_ZONE_START_POSE;
            }
        }

    }

    @Override
    public void runOpMode() {

        Initialize();
        waitForStart();

        sleep(1000);

        StartChecking();
        MakeDetection(5);



    }

    private void MakeDetection(int timeoutInSeconds) {
        int tries = 0;
        while (opModeIsActive() && !circleDetection.isDetected() && tries < timeoutInSeconds * 10) {
            sleep(100);
            tries++;
            telemetry.addData("Detection tries:", tries);
        }
        if (!circleDetection.isDetected()){
            spikePosition = "MID";
        } else{
            spikePosition = circleDetection.spikePosition;
        }
    }

    private void StartChecking(){
        circleDetection.setState(CircleDetectionPipeline.DetectionState.DETECT);
    }

    private Pose2d RED_BACKSTAGE_START_POSE = new Pose2d(60, 14, Math.toRadians(180));
    private Pose2d RED_LANDING_ZONE_START_POSE = new Pose2d(60, -38, Math.toRadians(180));
    private Pose2d BLUE_BACKSTAGE_START_POSE = new Pose2d(-60, 14, Math.toRadians(0));
    private Pose2d BLUE_LANDING_ZONE_START_POSE = new Pose2d(-60, -38, Math.toRadians(0));

    public TrajectorySequence BlueBackstageLeftTrajectory = drive.trajectorySequenceBuilder(startPose)
            .splineTo(new Vector2d(-36, 23), Math.toRadians(0))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    public TrajectorySequence BlueBackstageCenterTrajectory = drive.trajectorySequenceBuilder(startPose)
            .splineTo(new Vector2d(-30, 11), Math.toRadians(0))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    public TrajectorySequence BlueBackstageRightTrajectory = drive.trajectorySequenceBuilder(startPose)
            .splineToLinearHeading(new Pose2d(-32, 10, Math.toRadians(-90)), Math.toRadians(-90))
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

}
