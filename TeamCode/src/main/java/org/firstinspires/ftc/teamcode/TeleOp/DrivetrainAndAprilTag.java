/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.VisionFunctions;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.RoboMom;


@TeleOp(name="DriveTrain+AprilTag", group="Z")


public class DrivetrainAndAprilTag extends RoboMom {

    double deadZone = 0.5;

    double speedScalar = 1;


    private float TARGET_DISTANCE_TO_TAG = 2f;

     double STRAFE_GAIN = 0.0271;
     double FORWARD_GAIN = 0.0288;

     double ROTATION_GAIN = 0.022;

    double rightTriggerPull;

    double increment = 0.00001;

    private VisionFunctions vision;

    private boolean controlsRelinquished = false;



    private final double DEADZONE = 0.1;


    private DrivetrainFunctions drivetrainFunctions;



     private static final double HARDWARECHECK_DELAY = 1;

    private ElapsedTime hardwareCheckTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        super.runOpMode();

        vision = new VisionFunctions(this);
        drivetrainFunctions = new DrivetrainFunctions(this);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();


        while (opModeIsActive()) {

            if (drivetrainFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                drivetrainFunctions.Reinitialize();

            if (hardwareCheckTimer.seconds() >= HARDWARECHECK_DELAY)
                hardwareCheckTimer.reset();

            /**GAMEPAD 1**/

            //Store inputted desired velocity (left_stick_x, left_stick_y)
            if (!controlsRelinquished) {

                if (Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone) {
                    drivetrainFunctions.Move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, speedScalar);
                } else {
                    drivetrainFunctions.Stop();
                }
            }

            if(gamepad1.dpad_up){
                FORWARD_GAIN += increment;
            }
            if(gamepad1.dpad_down){
                FORWARD_GAIN -= increment;
            }

            if(gamepad1.dpad_left){
                STRAFE_GAIN -= increment;
            }
            if(gamepad1.dpad_right){
                STRAFE_GAIN += increment;
            }

            if(gamepad1.x){
                ROTATION_GAIN -= increment;
            }
            if(gamepad1.b){
                ROTATION_GAIN += increment;
            }

            telemetry.addData("Forward Gain: ", FORWARD_GAIN);
            telemetry.addData("Strafe Gain: ", STRAFE_GAIN);
            telemetry.addData("Rotation Gain: ", ROTATION_GAIN);


            if(vision.DetectAprilTag(vision.BLUE_1_TAG)){
                telemetry.addData("Found", "ID %d (%s)", vision.detectedTag.id, vision.detectedTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", vision.detectedTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", vision.detectedTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", vision.detectedTag.ftcPose.yaw);
                telemetry.addData("X delta","%3.0f inches", vision.detectedTag.ftcPose.x);

                if(gamepad1.right_trigger > 0.025f){
                    rightTriggerPull = gamepad1.right_trigger;

                    double x = STRAFE_GAIN * vision.detectedTag.ftcPose.yaw;
                    double y = -FORWARD_GAIN * vision.detectedTag.ftcPose.range;
                    double bearing = -ROTATION_GAIN * vision.detectedTag.ftcPose.bearing;

                    telemetry.addData("x: ", x);
                    telemetry.addData("y: ", y);
                    telemetry.addData("bearing: ", bearing);

                    if(vision.detectedTag.ftcPose.range > TARGET_DISTANCE_TO_TAG){
//                        y = Math.max(y,-0.05);
                        drivetrainFunctions.Move((float)x,(float)y,(float)bearing, 1);
                    } else{
                        drivetrainFunctions.Move(0,0,0, 0);
                    }

                } else {
                    controlsRelinquished = false;
                }
            }


            telemetry.update();


            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                speedScalar = 0.5;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            } else {
                speedScalar = 1;
            }



            //Gamepad 2


        }
    }

}
