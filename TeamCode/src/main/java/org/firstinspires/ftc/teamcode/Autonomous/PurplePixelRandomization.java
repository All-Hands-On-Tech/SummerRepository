package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.aprilTag.AprilTagDetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class PurplePixelRandomization extends RoboMom {

    public int randomization, finalRandomization;
    int PIXEL_THRESH = 100;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    OpenCvCamera webcam;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);




        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode){

            }
        });

        waitForStart();




        telemetry.addData("Randomization: ", randomization);
        telemetry.update();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            switch(randomization){
                case 1:
                    //LEFT
                    //We need to finish odometry and Roadrunner, for now we are using driveForTime
//                    driveForTime("LEFT", 0.3, 0.75);
//                    sleep(100);
//                    driveForTime("FORWARD", 0.3, 0.75);
//                    sleep(100);
//                    driveForTime("BACK", 0.3, 0.5);
//                    sleep(100);
                    //driveForTime("ROTATE_RIGHT", 0.3, 0.5);

                    telemetry.addLine("left");
                    break;
                case 2:
                    //MIDDLE
                    //replace with odometry and roadrunner
//                    driveForTime("FORWARD", 0.3, 1);
//                    sleep(100);
//                    driveForTime("BACK", 0.3, 0.5);

                    telemetry.addLine("center");
                    break;
                case 3:
                    //replace with odometry and roadrunner
                    //RIGHT
//                    driveForTime("RIGHT", 0.3, 0.75);
//                    sleep(100);
//                    driveForTime("FORWARD", 0.3, 0.75);
//                    sleep(100);
//                    driveForTime("BACK", 0.3, 0.5);

                    telemetry.addLine("right");
                    break;
            }

            telemetry.update();
        }
    }

//    class SamplePipeline extends OpenCvPipeline{
//        Mat HSVimage = new Mat();
//        double leftGreenPixels,midGreenPixels,rightGreenPixels;

//     class SamplePipeline extends OpenCvPipeline{
//        Mat HSVimage = new Mat();
//        double leftGreenPixels,midGreenPixels,rightGreenPixels;
//
//        boolean viewportPaused;
//        Scalar greenLower = new Scalar(30, 75, 75);
//        Scalar greenHigher = new Scalar(90, 255, 255);


      //  @Override
//        public Mat processFrame(Mat input) {
//            //convert to hsv
//            Imgproc.cvtColor(input, HSVimage, Imgproc.COLOR_RGB2HSV);
//            //create rect around bottom 2/3 of mat
//            Rect rectCropTop = new Rect(0, input.height()/3, input.width(), input.height()*(2/3));
//            //store mat ROI in cropped mat
//            Mat cropped = input.submat(rectCropTop);
//
//            //Create rects around first, second, third sections of cropped mat
//            Rect rectCropLeft = new Rect(0, 0, cropped.width()/3, cropped.height());
//            Rect rectCropMid = new Rect(cropped.width()/3, 0, cropped.width()/3, cropped.height());
//            Rect rectCropRight = new Rect(cropped.width()*(2/3), 0, cropped.width()/3, cropped.height());
//
//            //Store mat ROI in left/mid/rightROI
//            Mat leftROI = input.submat(rectCropLeft);
//            Mat midROI = input.submat(rectCropMid);
//            Mat rightROI = input.submat(rectCropRight);
//
//            //pixels in greenthreshold now = 1, outside thresh = 0
//            Core.inRange(leftROI, greenLower, greenHigher, leftROI);
//            Core.inRange(midROI, greenLower, greenHigher, midROI);
//            Core.inRange(rightROI, greenLower, greenHigher, rightROI);
//
//            //count pixels in ROI's
//            leftGreenPixels = Core.sumElems(leftROI).val[0];
//            midGreenPixels = Core.sumElems(midROI).val[0];
//            rightGreenPixels = Core.sumElems(rightROI).val[0];
//
//            //Check if ROI has enough green (left)
//            if(leftGreenPixels > PIXEL_THRESH){
//                randomization = 1;
//            }
//            //check right, else: mid
//            if(rightGreenPixels > leftGreenPixels && rightGreenPixels > PIXEL_THRESH){
//                randomization = 3;
//            } else {
//                randomization = 2;
//            }
//
//            return null;
//        }
//    }

}