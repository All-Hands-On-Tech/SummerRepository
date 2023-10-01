package org.firstinspires.ftc.teamcode.Vision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CheckPipeline extends OpenCvPipeline {

    public Rect rectLeft = VisionConstants.rectLeftROI;
    public Rect rectRight = VisionConstants.rectRightROI;
    public Rect rectMid = VisionConstants.rectMidROI;

    public Scalar low = VisionConstants.lowColorThreshold;
     public Scalar high = VisionConstants.highColorThreshold;
    Mat HSVimage = new Mat();
    Mat LeftBinaryMat = new Mat();
    Mat RightBinaryMat = new Mat();
    Mat MidBinaryMat = new Mat();

    Mat LeftMaskedMat = new Mat();
    Mat RightMaskedMat = new Mat();
    Mat MidMaskedMat = new Mat();
    Mat LeftROI = new Mat();
    Mat RightROI = new Mat();
    Mat MidROI = new Mat();

    @Override
    public void init(Mat firstFrame) {

    }

    @Override
    public Mat processFrame(Mat input) {

        LeftMaskedMat.release();
        RightMaskedMat.release();
        MidMaskedMat.release();

        LeftROI.release();
        RightROI.release();
        MidROI.release();

        Imgproc.cvtColor(input, HSVimage, Imgproc.COLOR_RGB2HSV);

        LeftROI = HSVimage.submat(rectLeft);
        RightROI = HSVimage.submat(rectRight);
        MidROI = HSVimage.submat(rectMid);

        //Mat submat = input.submat(rect);

        Core.inRange(LeftROI, low, high, LeftBinaryMat);
        Core.inRange(RightROI, low, high, RightBinaryMat);
        Core.inRange(MidROI, low, high, MidBinaryMat);

        Core.bitwise_and(LeftROI, LeftROI, LeftMaskedMat, LeftBinaryMat);
        Core.bitwise_and(RightROI, RightROI, RightMaskedMat, RightBinaryMat);
        Core.bitwise_and(MidROI, MidROI, MidMaskedMat, MidBinaryMat);

        LeftMaskedMat.copyTo(LeftROI);
        RightMaskedMat.copyTo(RightROI);
        MidMaskedMat.copyTo(MidROI);

        Imgproc.cvtColor(HSVimage, HSVimage, Imgproc.COLOR_HSV2RGB);

        return HSVimage;

    }

}