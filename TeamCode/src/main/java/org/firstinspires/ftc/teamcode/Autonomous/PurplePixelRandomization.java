package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.aprilTag.AprilTagDetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
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

    SubmatPipeline submatPipeline;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        submatPipeline = new SubmatPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(submatPipeline);




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

    class SubmatPipeline extends AprilTagDetectionPipeline {
        // Notice this is declared as an instance variable (and re-used), not a local variable
        Mat submat;

        public SubmatPipeline(double tagsize, double fx, double fy, double cx, double cy) {
            super(tagsize, fx, fy, cx, cy);
        }

        @Override
        public void init(Mat firstFrame) {

        }

        @Override
        public Mat processFrame(Mat firstFrame) {
            // Because a submat is a persistent reference to a region of the parent buffer,
            // (which in this case is `input`) any changes to `input` will be reflected in
            // the submat (and vice versa).
            int rows = firstFrame.rows();
            int ROIHeight = rows * (2/3);
            int ROIWidth = firstFrame.cols()/3;

            Rect leftROI = new Rect(0, rows/3, ROIWidth, ROIHeight);
            Rect midROI = new Rect(ROIWidth, rows/3, ROIWidth, ROIHeight);
            Rect rightROI = new Rect(ROIWidth * 2, rows/3, ROIWidth, ROIHeight);

            double leftPropPixels = PropPixelsInROI(firstFrame, leftROI);
            double midPropPixels = PropPixelsInROI(firstFrame, midROI);
            double rightPropPixels = PropPixelsInROI(firstFrame, rightROI);

            telemetry.addData("LeftPropPixels",leftPropPixels);
            telemetry.addData("MidPropPixels",midPropPixels);
            telemetry.addData("RightPropPixels",rightPropPixels);
            telemetry.update();

            if(leftPropPixels > midPropPixels && leftPropPixels > rightPropPixels){
                randomization = 1;
            } else if(midPropPixels > rightPropPixels){
                randomization = 2;
            } else{
                randomization = 3;
            }
            return firstFrame;
        }

        public double PropPixelsInROI(Mat input, Rect rect) {
            int PIXEL_THRESH = 100;

            Mat HSVimage = new Mat();
            double GreenPixels;

            boolean viewportPaused;
            Scalar greenLower = new Scalar(30, 75, 75);
            Scalar greenHigher = new Scalar(90, 255, 255);

            //convert to hsv
            Imgproc.cvtColor(input, HSVimage, Imgproc.COLOR_RGB2HSV);
            //create rect around bottom 2/3 of mat
            //store mat ROI in cropped mat

            //Store mat ROI in left/mid/rightROI
            Mat ROI = new Mat(input, rect);

            //pixels in greenthreshold now = 1, outside thresh = 0


            if (!ROI.empty()) {
                Core.inRange(ROI, greenLower, greenHigher, ROI);
            }
//            else{
//                Point point = new Point(10,10);
//                Imgproc.putText(input, "empty", point, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, green, 2);
//            }


            //count pixels in ROI's
            GreenPixels = Core.sumElems(ROI).val[0];

            //Check if ROI has enough green (left)

            return GreenPixels;
        }
    }

    }