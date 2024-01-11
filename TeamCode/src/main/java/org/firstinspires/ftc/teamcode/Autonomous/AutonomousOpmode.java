package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagsFunctions;
import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
public class AutonomousOpmode extends LinearOpMode {

    protected OpenCvWebcam webcam;

    protected double fx = VisionConstants.fx;
    protected double fy = VisionConstants.fy;
    protected double cx = VisionConstants.cx;
    protected double cy = VisionConstants.cy;
    protected int RESWIDTH = VisionConstants.RESWIDTH;
    protected int RESHEIGHT = VisionConstants.RESHEIGHT;
    protected int TIMEOUT = 5;
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
    protected AprilTagsFunctions aprilTagsFunctions;
    protected CircleDetectionPipeline circleDetectionPipeline;
    protected DeliveryFunctions deliveryFunctions;


    protected void Initialize(LinearOpMode l) {
        circleDetectionPipeline = new CircleDetectionPipeline(l.telemetry, isRed);

        aprilTagsFunctions = new AprilTagsFunctions(l);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(circleDetectionPipeline);

        drive = new SampleMecanumDrive(hardwareMap);
        deliveryFunctions = new DeliveryFunctions(l, true);
        aprilTagsFunctions = new AprilTagsFunctions(l);



    }

    protected String MakeDetection(int timeoutInSeconds) {
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

    //    if (aprilTagsFunctions.DetectAprilTag(aprilTagsFunctions.BLUE_1_TAG)) {
//        telemetry.addData("Found", "ID %d (%s)", aprilTagsFunctions.detectedTag.id, aprilTagsFunctions.detectedTag.metadata.name);
//        telemetry.addData("Range", "%5.1f inches", aprilTagsFunctions.detectedTag.ftcPose.range);
//        telemetry.addData("Bearing", "%3.0f degrees", aprilTagsFunctions.detectedTag.ftcPose.bearing);
//        telemetry.addData("Yaw", "%3.0f degrees", aprilTagsFunctions.detectedTag.ftcPose.yaw);
//        telemetry.addData("X delta", "%3.0f inches", aprilTagsFunctions.detectedTag.ftcPose.x);
//
//        if (gamepad1.right_trigger > 0.025f) {
//            rightTriggerPull = gamepad1.right_trigger;
//
////                    strafeGain *= rightTriggerPull;
////                    forwardGain *= rightTriggerPull;
////                    rotationGain *= rightTriggerPull;
//
//            double x = STRAFE_GAIN * aprilTagsFunctions.detectedTag.ftcPose.yaw;
//            double y = -FORWARD_GAIN * aprilTagsFunctions.detectedTag.ftcPose.range;
//
////                    double x = 0.5;
////                    double y = 0.7;
//            double bearing = -ROTATION_GAIN * aprilTagsFunctions.detectedTag.ftcPose.bearing;
//
//            telemetry.addData("x: ", x);
//            telemetry.addData("y: ", y);
//            telemetry.addData("bearing: ", bearing);
//
//            drivetrainFunctions.Move((float) x, (float) y, (float) bearing, 1);
//        } else {
//            controlsRelinquished = false;
//        }
//
//                /*
//                if (currentGamepad1.right_trigger > 0.5) {
//                    y      = SPEED_GAIN * (aprilTagsFunctions.detectedTag.ftcPose.range - DESIRED_DISTANCE_TO_APRIL_TAG_INCHES);
//                    yaw    = -TURN_GAIN * aprilTagsFunctions.detectedTag.ftcPose.yaw;
//                    x      = STRAFE_GAIN * aprilTagsFunctions.detectedTag.ftcPose.x;
//                    isAutoDrivingToAprilTag = true;
//                }
//                 */
//    } else {          //align to point (pose of aprilTag)
//
//    }
    public void runOpMode() {
        waitForStart();
    }
}
