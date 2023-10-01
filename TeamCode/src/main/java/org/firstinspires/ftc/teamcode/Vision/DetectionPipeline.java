package org.firstinspires.ftc.teamcode.Vision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DetectionPipeline extends OpenCvPipeline {

    Telemetry telemetry;

    public Rect rectLeft = VisionConstants.rectLeftROI;
    public Rect rectRight = VisionConstants.rectRightROI;
    public Rect rectMid = VisionConstants.rectMidROI;

    public Scalar low = VisionConstants.lowColorThreshold;
     public Scalar high = VisionConstants.highColorThreshold;
    Mat HSVimage = new Mat();
    Mat BinaryMatLeft = new Mat();
    Mat BinaryMatRight = new Mat();
    Mat BinaryMatMid = new Mat();
    Mat ROILeft = new Mat();
    Mat ROIRight = new Mat();
    Mat ROIMid = new Mat();

    String spikePosition = "MID";

    double pixLeft, pixRight, pixMid;


    public DetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat firstFrame) {

    }

    @Override
    public Mat processFrame(Mat input) {

        ROILeft.release();
        ROIRight.release();
        ROIMid.release();

        BinaryMatLeft.release();
        BinaryMatRight.release();
        BinaryMatMid.release();

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

    public String getSpikePosition(){
        if(spikePosition != null){
            return spikePosition;
        } else {
            return "RIGHT";
        }

    }

}