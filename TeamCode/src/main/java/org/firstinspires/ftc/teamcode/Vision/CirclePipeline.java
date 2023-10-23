package org.firstinspires.ftc.teamcode.Vision;


import android.text.BoringLayout;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
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

    Mat MaskedMat = new Mat();

    Mat BinaryMat = new Mat();
    Mat Overlay = new Mat();

    public float sigmaX = 1.5f;
    public float sigmaY = 1.5f;

    public double minDist = 150f;
    public double param1 = 130;
    public double param2 = 30;

    Boolean isRed;

    Scalar lowRed = VisionConstants.lowRedThreshold;
    Scalar highRed = VisionConstants.highRedThreshold;

    Scalar lowBlue = VisionConstants.lowBlueThreshold;
    Scalar highBlue = VisionConstants.highBlueThreshold;
    Size Kernel = new Size(7,7);


    public CirclePipeline(Telemetry telemetry, Boolean isRed) {
        this.telemetry = telemetry;
        this.isRed = isRed;
    }

    @Override
    public void init(Mat firstFrame) {

    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, HSVImage, Imgproc.COLOR_RGB2HSV);

        if(isRed){
            Core.inRange(HSVImage, lowRed, highRed, BinaryMat);
        } else{
            Core.inRange(HSVImage, lowBlue, highBlue, BinaryMat);
        }


        Core.bitwise_and(input, input, MaskedMat, BinaryMat);

        Imgproc.cvtColor(MaskedMat, GrayImage, Imgproc.COLOR_RGB2GRAY);

        Imgproc.GaussianBlur(GrayImage, Blur, Kernel, sigmaX, sigmaY);


        Imgproc.HoughCircles(Blur, Circles,Imgproc.CV_HOUGH_GRADIENT,  1, minDist, param1, param2);

        int numCircles = Circles.cols();

        MaskedMat.copyTo(Overlay);
        Point center = new Point(0, 0);


        for(int i=0; i < numCircles; i++)
        {
            double[] data = Circles.get(0, i);
            center = new Point(Math.round(data[0]), Math.round(data[1]));
            // circle center
            Imgproc.circle(Overlay, center, 1, new Scalar(0, 0, 255), 2, 8, 0 );
            // circle outline
            int radius = (int) Math.round(data[2]);
            Imgproc.circle(Overlay, center, radius, new Scalar(0,0,255), 2, 8, 0 );
        }

        return Overlay;

    }

}