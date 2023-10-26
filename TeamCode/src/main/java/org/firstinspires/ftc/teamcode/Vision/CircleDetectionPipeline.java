package org.firstinspires.ftc.teamcode.Vision;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CircleDetectionPipeline extends OpenCvPipeline {

    public String spikePosition = "MID";

    public Rect rect = VisionConstants.rectLowROI;

    Telemetry telemetry;
    Mat HSVImage = new Mat();
    Mat GrayImage = new Mat();
    Mat Blur = new Mat();
    Mat Circles = new Mat();

    Mat MaskedMat = new Mat();

    Mat BinaryMat = new Mat();
    Mat Overlay = new Mat();

    Mat ROI = new Mat();

    boolean isRed;

    public float sigmaX = 1.5f;
    public float sigmaY = 1.5f;

    public double minDist = 150f;
    public double param1 = 130;
    public double param2 = 30;
    public double x;

    Point center;

    Scalar lowRed = VisionConstants.lowRedThreshold;
    Scalar highRed = VisionConstants.highRedThreshold;

    Scalar lowBlue = VisionConstants.lowBlueThreshold;
    Scalar highBlue = VisionConstants.highBlueThreshold;

    Size Kernel = new Size(7,7);


    public CircleDetectionPipeline(Telemetry telemetry, boolean isRed) {
        this.telemetry = telemetry;
        this.isRed = isRed;
    }

    @Override
    public void init(Mat firstFrame) {
    }

    @Override
    public Mat processFrame(Mat input) {

        ROI.release();
        MaskedMat.release();
        Overlay.release();
        Circles.release();

        Imgproc.cvtColor(input, HSVImage, Imgproc.COLOR_RGB2HSV);

        ROI = HSVImage.submat(rect);

        if(isRed){
            Core.inRange(ROI, lowRed, highRed, BinaryMat);
        } else{
            Core.inRange(ROI, lowBlue, highBlue, BinaryMat);
        }


        Core.bitwise_and(ROI, ROI, MaskedMat, BinaryMat);

        Imgproc.cvtColor(MaskedMat, GrayImage, Imgproc.COLOR_RGB2GRAY);

        Imgproc.GaussianBlur(GrayImage, Blur, Kernel, sigmaX, sigmaY);


        Imgproc.HoughCircles(Blur, Circles,Imgproc.CV_HOUGH_GRADIENT,  1, minDist, param1, param2);

        int numCircles = Circles.cols();
        ROI.copyTo(Overlay);

        if(numCircles > 0){
            double[] data = Circles.get(0, 0);
            center = new Point(Math.round(data[0]), Math.round(data[1]));
            x = center.x;
        }


        Overlay.copyTo(ROI);


        if(x < rect.width/3){
            spikePosition = "LEFT";
        } else if(x >= rect.width/3 && x <= (rect.width*2)/3){
            spikePosition = "MID";
        } else if(x > (rect.width*2)/3){
            spikePosition = "RIGHT";
        } else{
            spikePosition ="MID";
        }

        return input;

    }

    public String getSpikePosition() {
        return spikePosition;
    }
}