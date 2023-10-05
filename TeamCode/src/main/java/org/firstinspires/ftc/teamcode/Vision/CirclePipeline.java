package org.firstinspires.ftc.teamcode.Vision;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CirclePipeline extends OpenCvPipeline {

    Telemetry telemetry;
    Mat HSVImage = new Mat();
    Mat GrayImage = new Mat();
    Mat Blur = new Mat();
    Mat Circles = new Mat();

    float sigmaX = 1.5f;
    float sigmaY = 1.5f;

    Size Kernel = new Size(7,7);


    public CirclePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat firstFrame) {

    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, HSVImage, Imgproc.COLOR_RGB2HSV);

        Imgproc.cvtColor(HSVImage, GrayImage, Imgproc.COLOR_RGB2GRAY);

        Imgproc.GaussianBlur(GrayImage, Blur, Kernel, sigmaX, sigmaY);


        Imgproc.HoughCircles(Blur, Circles,Imgproc.CV_HOUGH_GRADIENT,  1, 200, 130, 3);

        Circles.copyTo(HSVImage);

        return HSVImage;

    }

    public String getLatestSpikePosition(){
        if(spikePosition != null){
            return spikePosition;
        } else {
            return "RIGHT";
        }

    }

}