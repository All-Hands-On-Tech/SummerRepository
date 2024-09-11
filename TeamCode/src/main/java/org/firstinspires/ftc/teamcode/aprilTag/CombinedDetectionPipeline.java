package org.firstinspires.ftc.teamcode.aprilTag;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.firstinspires.ftc.teamcode.aprilTag.AprilTagDetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class CombinedDetectionPipeline extends AprilTagDetectionPipeline {

    public enum DetectionState {
        APRILTAG,
        TEAMPROP
    }

    DetectionState state = DetectionState.TEAMPROP;
    Telemetry telemetry;

    public Rect rectLeft = VisionConstants.rectLeftROI;
    public Rect rectRight = VisionConstants.rectRightROI;
    public Rect rectMid = VisionConstants.rectMidROI;

    public Scalar low = VisionConstants.lowRedThreshold;
     public Scalar high = VisionConstants.highRedThreshold;
    Mat HSVimage = new Mat();
    Mat BinaryMatLeft = new Mat();
    Mat BinaryMatRight = new Mat();
    Mat BinaryMatMid = new Mat();
    Mat ROILeft = new Mat();
    Mat ROIRight = new Mat();
    Mat ROIMid = new Mat();
    private Mat grey = new Mat();
    private Mat ret = new Mat();

    String spikePosition = "MID";

    double pixLeft, pixRight, pixMid;

    double fx;
    double fy;
    double cx;
    double cy;

    // UNITS ARE METERS
    double tagsize;
    double tagsizeX;
    double tagsizeY;

    private long nativeApriltagPtr;

    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    Scalar blue = new Scalar(7, 197, 235, 255);
    Scalar red = new Scalar(255, 0, 0, 255);
    Scalar green = new Scalar(0, 255, 0, 255);
    Scalar white = new Scalar(255, 255, 255, 255);

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();



    public CombinedDetectionPipeline(double tagsize, double fx, double fy, double cx, double cy) {
        super(tagsize, fx, fy, cx, cy);

        this.tagsize = tagsize;
        this.tagsizeX = tagsize;
        this.tagsizeY = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        constructMatrix();

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_16h5.string, 3, 3);
    }

    @Override
    public void init(Mat firstFrame) {

    }

    public void finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if (nativeApriltagPtr != 0) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        } else {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    @Override
    public Mat processFrame(Mat input) {

        switch (state){
            case TEAMPROP:
                ret = TeamPropDetect(input);
                return ret;
            case APRILTAG:
                ret = AprilTagDetect(input);
                return ret;
        }

        return input;
    }

    public String getLatestSpikePosition(){
        if(spikePosition != null){
            return spikePosition;
        } else {
            return "RIGHT";
        }

    }

    public void SetState(DetectionState newState){
        state = newState;
    }

    public ArrayList<AprilTagDetection> GetLatestDetections(){
        return detections;
    }

    public

    Mat TeamPropDetect(Mat input){
        ROILeft.release();
        ROIRight.release();
        ROIMid.release();

        BinaryMatLeft.release();
        BinaryMatRight.release();
        BinaryMatMid.release();

        HSVimage.release();

        Imgproc.cvtColor(input, HSVimage, Imgproc.COLOR_RGB2HSV);

        ROILeft = HSVimage.submat(rectLeft);
        ROIRight = HSVimage.submat(rectRight);
        ROIMid = HSVimage.submat(rectMid);

        Core.inRange(ROILeft, low, high, BinaryMatLeft);
        Core.inRange(ROIRight, low, high, BinaryMatRight);
        Core.inRange(ROIMid, low, high, BinaryMatMid);

        pixLeft = Core.sumElems(BinaryMatLeft).val[0];
        pixRight = Core.sumElems(BinaryMatRight).val[0];
        pixMid = Core.sumElems(BinaryMatMid).val[0];

        spikePosition = "Mid";

        if(pixLeft > pixRight && pixLeft > pixMid){
            spikePosition = "LEFT";
        }

        if(pixRight > pixLeft && pixRight > pixMid){
            spikePosition = "RIGHT";
        }

        if(pixMid > pixLeft && pixMid > pixRight){
            spikePosition = "MID";
        }

        telemetry.addData("Spike Position", spikePosition);
        telemetry.update();

        return BinaryMatMid;
    }

    Mat AprilTagDetect(Mat input){
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync) {
            if (needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }

        // For fun, use OpenCV to draw 6DOF markers on the image.
        for (AprilTagDetection detection : detections) {
            Pose pose = aprilTagPoseToOpenCvPose(detection.pose);
            //Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
            drawAxisMarker(input, tagsizeY / 2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);

        }

        return input;
    }
}