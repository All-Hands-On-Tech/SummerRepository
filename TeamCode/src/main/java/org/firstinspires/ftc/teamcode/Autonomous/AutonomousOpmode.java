package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.Competition.BlueLandingZone;
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

    protected TrajectorySequence left;
    protected TrajectorySequence center;
    protected TrajectorySequence right;

    protected AutonomousTrajectories trajectories;


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

        SetAppropriateTrajectories();

    }

    @Override
    public void runOpMode() {

        Initialize();
        waitForStart();

        sleep(1000);

        StartChecking();
        MakeDetection(5);


        switch (spikePosition) {
            case "LEFT":
                telemetry.addLine("left");
                telemetry.update();
                drive.followTrajectorySequence(left);
                break;
            case "MID":
                telemetry.addLine("center");
                telemetry.update();
                drive.followTrajectorySequence(center);
                break;
            case "RIGHT":
                telemetry.addLine("right");
                telemetry.update();
                drive.followTrajectorySequence(right);
                break;
        }

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

    private void SetAppropriateTrajectories(){
        if(isRed){
            if(isBackstage){
                left = trajectories.RedBackstageLeftTrajectory;
                center = trajectories.RedBackstageCenterTrajectory;
                right = trajectories.RedBackstageRightTrajectory;
            }else{
                left = trajectories.RedLandingZoneLeftTrajectory;
                center = trajectories.RedLandingZoneCenterTrajectory;
                right = trajectories.RedLandingZoneRightTrajectory;

            }
        } else{
            if(isBackstage){
                left = trajectories.BlueBackstageLeftTrajectory;
                center = trajectories.BlueBackstageCenterTrajectory;
                right = trajectories.BlueBackstageRightTrajectory;
            } else{
                left = trajectories.BlueLandingZoneLeftTrajectory;
                center = trajectories.BlueLandingZoneCenterTrajectory;
                right = trajectories.BlueLandingZoneRightTrajectory;
            }

        }
    }

}
