package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


import org.firstinspires.ftc.teamcode.RoboMom;

import org.firstinspires.ftc.teamcode.Vision.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class CameraTesting extends RoboMom {

    public String randomization;
    int PIXEL_THRESH = 100;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    int RESWIDTH = 1920;
    int RESHEIGHT = 1080;

    // UNITS ARE METERS
    double tagsize = 0.166;
    OpenCvCamera webcam;

    DetectionPipeline detectionPipeline = new DetectionPipeline(telemetry);

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(detectionPipeline);




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




        telemetry.addData("Randomization: ", randomization);
        telemetry.update();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            randomization = detectionPipeline.getLatestSpikePosition();
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


            telemetry.update();
        }
    }


    }