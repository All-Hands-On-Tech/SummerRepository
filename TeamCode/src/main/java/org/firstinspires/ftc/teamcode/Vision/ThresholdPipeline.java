package org.firstinspires.ftc.teamcode.Vision;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ThresholdPipeline extends OpenCvPipeline {

    public Rect rect = VisionConstants.rectLowROI;

    public Scalar lowRed = VisionConstants.lowRedThreshold;
     public Scalar highRed = VisionConstants.highRedThreshold;
    public Scalar lowBlue = VisionConstants.lowBlueThreshold;
    public Scalar highBlue = VisionConstants.highBlueThreshold;

    boolean isRed;
    Mat HSVimage = new Mat();
    Mat BinaryMat = new Mat();

    Mat maskedMat = new Mat();
    Mat ROI = new Mat();

    public ThresholdPipeline(Boolean isRed) {
        this.isRed = isRed;
    }

    @Override
    public void init(Mat firstFrame) {

    }

    @Override
    public Mat processFrame(Mat input) {

        maskedMat.release();
        ROI.release();

        //convert to hsv
        Imgproc.cvtColor(input, HSVimage, Imgproc.COLOR_RGB2HSV);


        //Create submat of HSVimage
        ROI = HSVimage.submat(rect);
        //Mat submat = input.submat(rect);

        //threshold the ROI and put in BinaryMat matrix
        if(isRed){
            Core.inRange(ROI, lowRed, highRed, BinaryMat);
        } else{
            Core.inRange(ROI, lowBlue, highBlue, BinaryMat);
        }


        //Put pixels from ROI into maskedmat based on binaryMat mask
        Core.bitwise_and(ROI, ROI, maskedMat, BinaryMat);

        //replace ROI pixels with masked mat pixels
        maskedMat.copyTo(ROI);

        //Convert hsvimage back to rgb so it's easier to see things on the screen
        Imgproc.cvtColor(HSVimage, HSVimage, Imgproc.COLOR_HSV2RGB);

        return HSVimage;

    }

}