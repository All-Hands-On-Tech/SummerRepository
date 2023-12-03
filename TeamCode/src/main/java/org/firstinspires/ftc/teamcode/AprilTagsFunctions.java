package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagsFunctions {
    private LinearOpMode linearOpMode;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public final int RED_1_TAG = 4;
    public final int RED_2_TAG = 5;
    public final int RED_3_TAG = 6;

    public final int BLUE_1_TAG = 1;
    public final int BLUE_2_TAG = 2;
    public final int BLUE_3_TAG = 3;

    public AprilTagDetection detectedTag = null;

    public List<AprilTagDetection> currentDetections = null;

    public AprilTagsFunctions(LinearOpMode l){
        this.linearOpMode = l;
        Initialize();
    }

    private void Initialize()
    {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(linearOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
    }

    private void updateDetections() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    }
    public int numberOfDetections() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        return currentDetections.size();
    }


    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            linearOpMode.telemetry.addData("Camera", "Waiting");
            linearOpMode.telemetry.update();
            while (!linearOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                linearOpMode.sleep(20);
            }
            linearOpMode.telemetry.addData("Camera", "Ready");
            linearOpMode.telemetry.update();
        }
        // Set camera controls unless we are stopping.
        if (!linearOpMode.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                linearOpMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            linearOpMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            linearOpMode.sleep(20);
        }
    }

    public boolean DetectAprilTag(int desiredTag) {
        boolean targetFound = false;
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredTag < 0) || (detection.id == desiredTag)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    detectedTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    linearOpMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                linearOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        linearOpMode.telemetry.update();
        return targetFound;
    }

    public Pose2d absolutePositionFromAprilTag(AprilTagDetection aprilTag) {
        double tagX = 0;
        double tagY = 0;
        double tagAngle = 0;

        //add stuff here based on tag number
        //Needs to be checked
        switch (aprilTag.id) {
            case 1:
                tagX = -41.40;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 2:
                tagX = -35.40;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 3:
                tagX = -29.40;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 4:
                tagX = 29.48;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 5:
                tagX = 35.48;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 6:
                tagX = 41.48;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 7:
                tagX = 40.93;
                tagY = -70.58;
                tagAngle = -90;
                break;
            case 8:
                tagX = 35.43;
                tagY = -70.58;
                tagAngle = -90;
                break;
            case 9:
                tagX = -35.51;
                tagY = -70.58;
                tagAngle = -90;
            case 10:
                tagX = -41.01;
                tagY = -70.58;
                tagAngle = -90;
                break;
        }

        double range = aprilTag.ftcPose.range;
        double yaw = -Math.toRadians(aprilTag.ftcPose.yaw);
        double bearing = Math.toRadians(aprilTag.ftcPose.bearing);

        //add stuff here based on camera position relative to center of robot
        double x1 = 2;
        double y1 = 7.5;
        tagX-=x1*Math.cos(yaw)-y1*Math.sin(yaw);
        tagY-=x1*Math.sin(yaw)+y1*Math.cos(yaw);

        double x = tagX + range * Math.sin(yaw + bearing);
        double y = tagY - range * Math.cos(yaw + bearing);
        double angle = Math.toDegrees(yaw) + 90;

        return new Pose2d(x, y, Math.toRadians(angle));
    }

    public Pose2d AverageAbsolutePositionFromAprilTags() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double numOfDetections = numberOfDetections();
        double AprilTagX = 0;
        double AprilTagY = 0;
        double AprilTagAngle = 0;

        if (numOfDetections > 1) {
            for (AprilTagDetection detection : currentDetections) {
                Pose2d location = absolutePositionFromAprilTag(detection);
                AprilTagX += location.getX();
                AprilTagY += location.getY();
                AprilTagAngle += location.getHeading();
            }

            AprilTagX /= numOfDetections;
            AprilTagY /= numOfDetections;
            AprilTagAngle /= numOfDetections;

            return new Pose2d(AprilTagX, AprilTagY, AprilTagAngle);
        } else {
            return new Pose2d(0/0, 0/0, 0/0);
        }
    }

}


/*

package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagsFunctions {
    private LinearOpMode lom = null;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    public static final int TAG_BLUE_LEFT = 1;
    public static final int TAG_BLUE_CENTER = 2;
    public static final int TAG_BLUE_RIGHT = 3;
    public static final int TAG_RED_LEFT = 4;
    public static final int TAG_RED_CENTER = 5;
    public static final int TAG_RED_RIGHT = 6;
    // Used to hold the data for a detected AprilTag
    public AprilTagDetection detectedTag = null;
    public AprilTagsFunctions(LinearOpMode l)
    {
        lom = l;
        Initialize();
    }
    public boolean DetectAprilTag(int desiredTag)
    {
        boolean targetFound = false;
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredTag < 0) || (detection.id == desiredTag)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    detectedTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    lom.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                lom.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        lom.telemetry.update();
        return targetFound;
    }
    private void Initialize()
    {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(lom.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
    }
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */

    /*
private void setManualExposure(int exposureMS, int gain) {
    // Wait for the camera to be open, then use the controls
    if (visionPortal == null) {
        return;
    }
    // Make sure camera is streaming before we try to set the exposure controls
    if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        lom.telemetry.addData("Camera", "Waiting");
        lom.telemetry.update();
        while (!lom.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            lom.sleep(20);
        }
        lom.telemetry.addData("Camera", "Ready");
        lom.telemetry.update();
    }
    // Set camera controls unless we are stopping.
    if (!lom.isStopRequested())
    {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            lom.sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        lom.sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        lom.sleep(20);
    }
}
}



 */
