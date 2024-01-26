package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.VisionFunctions;
import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.IntakeFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
public class AutonomousOpmode extends LinearOpMode {

//    protected OpenCvWebcam webcam;

    protected double fx = VisionConstants.fx;
    protected double fy = VisionConstants.fy;
    protected double cx = VisionConstants.cx;
    protected double cy = VisionConstants.cy;
    protected int RESWIDTH = VisionConstants.RESWIDTH;
    protected int RESHEIGHT = VisionConstants.RESHEIGHT;
    protected int TIMEOUT = 5;
//    protected CircleDetectionPipeline circleDetection;
//    protected SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    protected Pose2d startPose;
    protected boolean isRed;
    protected boolean isBackstage;
    protected String spikePosition;

    protected TrajectorySequence left;
    protected TrajectorySequence center;
    protected TrajectorySequence right;

//    protected AutonomousTrajectories trajectories;
    protected VisionFunctions visionFunctions;
    protected CircleDetectionPipeline circleDetectionPipeline;
    protected DeliveryFunctions deliveryFunctions;
    protected IntakeFunctions intakeFunctions;

    protected ElapsedTime time;
    protected DrivetrainFunctions drivetrainFunctions;


    protected void Initialize(LinearOpMode l, boolean AllianceColorisRed, boolean startBackstage) {
        isRed = AllianceColorisRed;
        isBackstage = startBackstage;

        time = new ElapsedTime();
        time.startTime();
//        circleDetectionPipeline = new CircleDetectionPipeline(l.telemetry, isRed);

        intakeFunctions = new IntakeFunctions(l);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

//        webcam.setPipeline(circleDetectionPipeline);
//        drive = new SampleMecanumDrive(hardwareMap);

        deliveryFunctions = new DeliveryFunctions(l, true);
        visionFunctions = new VisionFunctions(l);

        drivetrainFunctions = new DrivetrainFunctions(l);

        visionFunctions.setRobotStartPosition(isRed, isBackstage);

        BeginPropDetection();
    }

    protected String MakePropDetection(int timeoutInSeconds) {
        telemetry.addData("Is Prop detected?", visionFunctions.checkIfPropIsDetected());
//        return visionFunctions.DetectTeamProp();

        BeginPropDetection();
        int tries = 0;
        while (opModeIsActive() && !visionFunctions.checkIfPropIsDetected() && tries < timeoutInSeconds * 10) {


            sleep(50);
            tries++;
        }
        if (!visionFunctions.checkIfPropIsDetected()){
            telemetry.addLine("Defaulted");
            BeginAprilTagDetection();
            telemetry.addData("Detection tries:", tries);
            telemetry.addData("Elapsed Time", time.seconds());
            return "MID";
        } else{
            BeginAprilTagDetection();
            telemetry.addData("Detection tries:", tries);
            telemetry.addData("Elapsed Time", time.seconds());
            return visionFunctions.DetectTeamProp();
        }
    }

    protected void BeginAprilTagDetection(){
        visionFunctions.startDetectingApriltags();
    }
    protected void BeginPropDetection(){
        visionFunctions.startDetectingProp();
    }

    protected void MoveToTagForSeconds(int desiredTag, double seconds){
        time.reset();
        while(time.seconds() < seconds){
            float moveProfile[] = visionFunctions.moveToTag(desiredTag);
            drivetrainFunctions.Move(moveProfile[0], moveProfile[1], moveProfile[2], 1);
        }
    }
    public void runOpMode() {
        waitForStart();
    }
}
