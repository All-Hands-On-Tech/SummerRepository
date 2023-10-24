package org.firstinspires.ftc.teamcode.Autonomous.Checks;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.Vision.ThresholdPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Check Blue Threshold", group="B")
public class BlueThresholdOpmode extends RoboMom {

    double fx = VisionConstants.fx;
    double fy = VisionConstants.fy;
    double cx = VisionConstants.cx;
    double cy = VisionConstants.cy;

    int RESWIDTH = VisionConstants.RESWIDTH;
    int RESHEIGHT = VisionConstants.RESHEIGHT;
    OpenCvCamera webcam;

    ThresholdPipeline thresholdPipeline = new ThresholdPipeline(false);

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(thresholdPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            public void onOpened()
            {
                webcam.startStreaming(RESWIDTH, RESHEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode){

            }
        });

        waitForStart();
//        webcam.pauseViewport();
        //webcam.resumeViewport();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }
    }


    }