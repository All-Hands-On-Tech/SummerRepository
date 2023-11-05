package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This 2023-2024 OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "AprilTagTestingNewAndImproved", group = "Concept")
public class AprilTagTestingNewAndImproved extends RoboMom {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    Boolean isRed = null;
    boolean isCameraOn = true;

    @Override
    public void runOpMode() {

        initAprilTag();

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //teemetry
            telemetry.addLine("Are the motors on: "+String.valueOf(areMotorsOn()));
            telemetry.addLine(String.format("XYH %6.1f %6.1f %6.1f (in, in, deg)", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading())));
            telemetry.update();

            //Only streams when the robot isn't moving and dpad up is pressed
            if (areMotorsOn()) {
                visionPortal.stopStreaming();
                isCameraOn = false;
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
                isCameraOn = true;
            }

            //when the camera is streaming the roadrunner pose is set to the apriltag pose
            //also checks which side of the field the robot is on
            if (isCameraOn) {
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    int numberOfDetections = currentDetections.size();
                    double AprilTagX = 0;
                    double AprilTagY = 0;
                    double AprilTagAngle = 0;

                    telemetry.addData("# AprilTags Detected", numberOfDetections);

                    // Step through the list of detections and display info for each one.
                    for (AprilTagDetection detection : currentDetections) {
                        Pose2d location = absolutePositionFromAprilTag(detection);
                        AprilTagX += location.getX();
                        AprilTagY += location.getY();
                        AprilTagAngle += location.getHeading();
                        if (detection.id==1 || detection.id==2 || detection.id==3) {
                            isRed = false;
                        } else if (detection.id==4 || detection.id==5 || detection.id==6) {
                            isRed = true;
                        }
                    }

                    AprilTagX /= numberOfDetections;
                    AprilTagY /= numberOfDetections;
                    AprilTagAngle /= numberOfDetections;

                    drive.setPoseEstimate(new Pose2d(AprilTagX, AprilTagY, AprilTagAngle));
                }

            // runs if robot is not currently following a trajectory
            //a is the tag on the left
            //b is the tag in the center
            //c is the tag on the right
            if (!drive.isBusy()) {
                double tagX = -41.40;
                double tagY = 48;
                double tagHeading = Math.toRadians(90);

                if (gamepad1.a) {
                    //this is the default
                } else if (gamepad1.b) {
                    tagX += 6;
                } else if (gamepad1.x) {
                    tagX += 12;
                }

                if (isRed) {
                    tagX += 70.88;
                }

                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(tagX, tagY, tagHeading))
                        .build()
                );
            }
            //runs if robots is following a trajectory a button is pressed to continue it
            else if(gamepad1.a || gamepad1.b || gamepad1.x) {
                drive.update();
            }
            //ends trajectory if no buttons are pressed
            else {
                drive.breakFollowing();
            }

            }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                //.setLensIntrinsics(1473.69, 1473.69, 845.856, 514.97)
                //.setLensIntrinsics(1529.40, 1529.40,1155.22, 576.677)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

}   // end class
