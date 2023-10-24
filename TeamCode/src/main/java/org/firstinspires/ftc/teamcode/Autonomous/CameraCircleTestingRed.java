package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class CameraCircleTestingRed extends RoboMom {

    public String randomization;
    int PIXEL_THRESH = 100;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    int RESWIDTH = VisionConstants.RESWIDTH;
    int RESHEIGHT = VisionConstants.RESHEIGHT;

    // UNITS ARE METERS
    double tagsize = 0.166;
    OpenCvCamera webcam;

    CircleDetectionPipeline circleDetectionPipeline = new CircleDetectionPipeline(telemetry, true);

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(circleDetectionPipeline);




        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            public void onOpened()
            {
                webcam.startStreaming(RESWIDTH, RESHEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode){

            }
        });

        waitForStart();

        //webcam.resumeViewport();



        if (isStopRequested()) return;

        String initialRandomization = circleDetectionPipeline.getSpikePosition();

        telemetry.addData("Randomization: ", initialRandomization);
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {

            randomization = circleDetectionPipeline.getSpikePosition();
            if(randomization != null){

                switch(randomization){
                    case "LEFT":
                        //LEFT
                        //We need to finish odometry and Roadrunner, for now we are using driveForTime
//                    driveForTime("LEFT", 0.3, 0.75);
//                    sleep(100);
//                    driveForTime("FORWARD", 0.3, 0.75);
//                    sleep(100);
//                    driveForTime("BACK", 0.3, 0.5);
//                    sleep(100);
                        //driveForTime("ROTATE_RIGHT", 0.3, 0.5);

                        telemetry.addLine("left");
                        break;
                    case "MID":
                        //MIDDLE
                        //replace with odometry and roadrunner
//                    driveForTime("FORWARD", 0.3, 1);
//                    sleep(100);
//                    driveForTime("BACK", 0.3, 0.5);

                        telemetry.addLine("center");
                        break;
                    case "RIGHT":
                        //replace with odometry and roadrunner
                        //RIGHT
//                    driveForTime("RIGHT", 0.3, 0.75);
//                    sleep(100);
//                    driveForTime("FORWARD", 0.3, 0.75);
//                    sleep(100);
//                    driveForTime("BACK", 0.3, 0.5);

                        telemetry.addLine("right");
                        break;
                }

            }

            telemetry.addData("Randomization: ", initialRandomization);
            telemetry.update();
        }
    }


    }