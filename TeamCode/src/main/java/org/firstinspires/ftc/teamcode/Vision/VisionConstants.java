package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class VisionConstants {

    static public Rect rectLeftROI = new Rect(20, 550, 550, 450);
    static public Rect rectRightROI = new Rect(1150, 550, 600, 450);
    static public Rect rectMidROI = new Rect(590, 450, 550, 450);

    static public Scalar lowColorThreshold = new Scalar(0, 100, 50);
    static public Scalar highColorThreshold = new Scalar(50, 255, 255);

    static public int RESWIDTH = 1920;
    static public int RESHEIGHT = 1080;

    static public double fx = 578.272;
    static public double fy = 578.272;
    static public double cx = 402.145;
    static public double cy = 221.506;
}
