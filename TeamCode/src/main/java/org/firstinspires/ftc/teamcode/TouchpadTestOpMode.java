package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This OpMode illustrates using the touchpad feature found on some gamepads.
 *
 * The Sony PS4 gamepad can detect two distinct touches on the central touchpad.
 * Other gamepads with different touchpads may provide mixed results.
 *
 * The touchpads are accessed through the standard gamepad1 and gamepad2 objects.
 *   Several new members were added to the Gamepad class in FTC SDK Rev 7
 *
 *   .touchpad_finger_1     returns true if at least one finger is detected.
 *   .touchpad_finger_1_x   finger 1 X coordinate.  Valid if touchpad_finger_1 is true
 *   .touchpad_finger_1_y   finger 1 Y coordinate.  Valid if touchpad_finger_1 is true
 *
 *   .touchpad_finger_2     returns true if a second finger is detected
 *   .touchpad_finger_2_x   finger 2 X coordinate.  Valid if touchpad_finger_2 is true
 *   .touchpad_finger_2_y   finger 2 Y coordinate.  Valid if touchpad_finger_2 is true
 *
 * Finger touches are reported with an X and Y coordinate in following coordinate system.
 *
 *   1) X is the Horizontal axis, and Y is the vertical axis
 *   2) The 0,0 origin is at the center of the touchpad.
 *   3)  1.0, 1.0 is at the top right corner of the touchpad.
 *   4) -1.0,-1.0 is at the bottom left corner of the touchpad.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */


@TeleOp(name="Touchpad Test", group ="Concept")
public class TouchpadTestOpMode extends LinearOpMode
{

    private double deadZone = 0.1;
    @Override
    public void runOpMode()
    {
        TouchpadFunctions touchpad1 = new TouchpadFunctions(this, 1);
        VisionFunctions vision = new VisionFunctions(this);
        DrivetrainFunctions drivetrainFunctions = new DrivetrainFunctions(this);

//        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addData(">", "Press Start");
        telemetry.update();

        waitForStart();

        vision.startDetectingApriltags();

        while (opModeIsActive())
        {
            boolean finger = false;

            touchpad1.CollectPreviousInput();
            touchpad1.CollectCurrentInput();

            if(vision.DetectAprilTag(-1)) {
                telemetry.addData("yaw ", vision.detectedTag.ftcPose.yaw);
                if (touchpad1.getTouchpad()) {
                    boolean down = touchpad1.getTouchpadDown();
                    boolean up = touchpad1.getTouchpadUp();
                    float xSwipe = touchpad1.getXSwipe();
                    float ySwipe = touchpad1.getYSwipe();
                    double rotation = -0.022 * vision.detectedTag.ftcPose.yaw;
                    double x = touchpad1.TOUCHPADXMULTIPLIER * xSwipe;
                    double y = -touchpad1.TOUCHPADYMULTIPLIER * ySwipe;


                    telemetry.addData("Touchpad Down? ", down);
                    telemetry.addData("Touchpad Up? ", up);
                    telemetry.addData("Touchpad Swipe:  ", xSwipe);
                    telemetry.addData("Rotation: ", rotation);
                    telemetry.addData("Strafe: ", x);
                    telemetry.addData("Forward: ", y);

                    double rotationDir = rotation / Math.abs(rotation);

                    if(Math.abs(rotation) > 0.5){
                        rotation = 0.5 * rotationDir;
                    }

                    if(Math.abs(rotation) < 0.15){
                        rotation = 0.15 * rotationDir;
                    }

                    drivetrainFunctions.Move((float) x, (float) y, (float) rotation, 1);
                } else {
                    drivetrainFunctions.Move(0, 0, 0, 1);
                }
            }

            if (Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone) {
                drivetrainFunctions.Move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1);
            } else {
                drivetrainFunctions.Stop();
            }


            telemetry.update();
            sleep(10);
        }
    }
}
