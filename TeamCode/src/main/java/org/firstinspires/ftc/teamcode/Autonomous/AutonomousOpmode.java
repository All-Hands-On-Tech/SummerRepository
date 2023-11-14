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

    public String MakeDetection(int timeoutInSeconds) {
        int tries = 0;
        while (opModeIsActive() && !circleDetection.isDetected() && tries < timeoutInSeconds * 10) {
            sleep(100);
            tries++;
            telemetry.addData("Detection tries:", tries);
        }
        if (!circleDetection.isDetected()){
            return "MID";
        } else{
            return circleDetection.spikePosition;
        }
    }

    private void StartChecking(){
        circleDetection.setState(CircleDetectionPipeline.DetectionState.DETECT);
    }

    private void SetAppropriateTrajectories(){
        if(isRed){
            if(isBackstage){
                left = RedBackstageLeftTrajectory;
                center = RedBackstageCenterTrajectory;
                right = RedBackstageRightTrajectory;
            }else{
                left = RedLandingZoneLeftTrajectory;
                center = RedLandingZoneCenterTrajectory;
                right = RedLandingZoneRightTrajectory;

            }
        } else{
            if(isBackstage){
                left = BlueBackstageLeftTrajectory;
                center = BlueBackstageCenterTrajectory;
                right = BlueBackstageRightTrajectory;
            } else{
                left = BlueLandingZoneLeftTrajectory;
                center = BlueLandingZoneCenterTrajectory;
                right = BlueLandingZoneRightTrajectory;
            }

        }
    }


    private Pose2d RED_BACKSTAGE_START_POSE = new Pose2d(60, 14, Math.toRadians(180));
    private Pose2d RED_LANDING_ZONE_START_POSE = new Pose2d(60, -38, Math.toRadians(180));
    private Pose2d BLUE_BACKSTAGE_START_POSE = new Pose2d(-60, 14, Math.toRadians(0));
    private Pose2d BLUE_LANDING_ZONE_START_POSE = new Pose2d(-60, -38, Math.toRadians(0));


    //BLUE BACKSTAGE
    public TrajectorySequence BlueBackstageLeftTrajectory = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(-36, 23), Math.toRadians(0))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    public TrajectorySequence BlueBackstageCenterTrajectory = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(-30, 11), Math.toRadians(0))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    public TrajectorySequence BlueBackstageRightTrajectory = drive.trajectorySequenceBuilder(BLUE_BACKSTAGE_START_POSE)
            .splineToLinearHeading(new Pose2d(-32, 10, Math.toRadians(-90)), Math.toRadians(-90))
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();


    //BLUE LANDING ZONE
    public TrajectorySequence BlueLandingZoneRightTrajectory = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineToConstantHeading(new Vector2d(-38, -47), Math.toRadians(0))
            .back(7)
            .strafeLeft(12.5)
            .forward(33)
            .lineToLinearHeading(new Pose2d(-8, 50, Math.toRadians(0)))
            .build();

    public TrajectorySequence BlueLandingZoneCenterTrajectory = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineTo(new Vector2d(-31, -35), Math.toRadians(0))
            .back(5)
            .strafeRight(15)
            .forward(25)
            .lineToLinearHeading(new Pose2d(-10, 50, Math.toRadians(0)))
            .build();

    public TrajectorySequence BlueLandingZoneLeftTrajectory = drive.trajectorySequenceBuilder(BLUE_LANDING_ZONE_START_POSE)
            .splineToLinearHeading(new Pose2d(-30, -34, Math.toRadians(90)), Math.toRadians(90))
            .back(5)
            .strafeRight(18)
            .lineToLinearHeading(new Pose2d(-12, 50, Math.toRadians(90)))
            .build();




    //RED BACKSTAGE
    public TrajectorySequence RedBackstageLeftTrajectory = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .splineToLinearHeading(new Pose2d(29, 10, Math.toRadians(-90)), Math.toRadians(-90))
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    public TrajectorySequence RedBackstageCenterTrajectory = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(30, 11), Math.toRadians(180))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();

    public TrajectorySequence RedBackstageRightTrajectory = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START_POSE)
            .splineTo(new Vector2d(36, 23), Math.toRadians(180))
            .back(10)
            .waitSeconds(1)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(55, 50, Math.toRadians(-90)), Math.toRadians(90))
            .build();


    //RED LANDING ZONE
    public TrajectorySequence RedLandingZoneLeftTrajectory = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .splineToConstantHeading(new Vector2d(38, -47), Math.toRadians(180))
            .back(7)
            .strafeRight(12.5)
            .forward(33)
            .lineToLinearHeading(new Pose2d(8, 50, Math.toRadians(180)))
            .build();

    public TrajectorySequence RedLandingZoneCenterTrajectory = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .splineTo(new Vector2d(31, -35), Math.toRadians(180))
            .back(5)
            .strafeLeft(15)
            .forward(27)
            .turn(Math.toRadians(90))
            .back(95)
            .build();

    public TrajectorySequence RedLandingZoneRightTrajectory = drive.trajectorySequenceBuilder(RED_LANDING_ZONE_START_POSE)
            .splineToLinearHeading(new Pose2d(30, -34, Math.toRadians(90)), Math.toRadians(90))
            .back(5)
            .strafeLeft(18)
            .lineToLinearHeading(new Pose2d(12, 50, Math.toRadians(90)))
            .build();

}
