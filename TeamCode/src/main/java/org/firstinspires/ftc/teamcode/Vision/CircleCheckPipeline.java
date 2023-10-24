package org.firstinspires.ftc.teamcode.Vision;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CircleCheckPipeline extends OpenCvPipeline {

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

    public float sigmaX = 1.5f;
    public float sigmaY = 1.5f;

    public double minDist = 150f;
    public double param1 = 130;
    public double param2 = 30;

    Scalar low = VisionConstants.lowRedThreshold;
    Scalar high = VisionConstants.highRedThreshold;

    Size Kernel = new Size(7,7);


    public CircleCheckPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat firstFrame) {
    }

    @Override
    public Mat processFrame(Mat input) {

        ROI.release();
        MaskedMat.release();
        Overlay.release();

        Imgproc.cvtColor(input, HSVImage, Imgproc.COLOR_RGB2HSV);

        ROI = HSVImage.submat(rect);

        Core.inRange(ROI, low, high, BinaryMat);

        Core.bitwise_and(ROI, ROI, MaskedMat, BinaryMat);

        Imgproc.cvtColor(MaskedMat, GrayImage, Imgproc.COLOR_RGB2GRAY);

        Imgproc.GaussianBlur(GrayImage, Blur, Kernel, sigmaX, sigmaY);


        Imgproc.HoughCircles(Blur, Circles,Imgproc.CV_HOUGH_GRADIENT,  1, minDist, param1, param2);

        int numCircles = Circles.cols();

        ROI.copyTo(Overlay);
        Point center;


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


        Overlay.copyTo(ROI);


        Imgproc.cvtColor(HSVImage, HSVImage, Imgproc.COLOR_HSV2RGB);
        Imgproc.rectangle(HSVImage, rect,new Scalar(0,255,0), 5, 8);

        return HSVImage;

    }

}