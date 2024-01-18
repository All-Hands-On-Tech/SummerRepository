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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.DroneLauncherFunctions;
import org.firstinspires.ftc.teamcode.IntakeFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.AprilTagsFunctions;

import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="CenterStage Teleop", group="AAA")


public class CenterStageTeleOp extends RoboMom {

    double deadZone = 0.05;

    double speedScalar = 1;


    private float TARGET_DISTANCE_TO_TAG = 12;

    final double STRAFE_GAIN = 0.0271;
    final double FORWARD_GAIN = 0.0288;

    final double ROTATION_GAIN = 0.022;

    double rightTriggerPull;

    private AprilTagsFunctions aprilTagsFunctions;

    private boolean controlsRelinquished = false;

    public enum DeliveryState{
        DELIVERY_START,
        DELIVERY_LIFT,
        DELIVERY_DUMP,
        DELIVERY_RETRACT
    }

    private final double DEADZONE = 0.1;

    private DeliveryFunctions deliveryFunctions;
    private IntakeFunctions intakeFunctions;

    private DrivetrainFunctions drivetrainFunctions;

    private DroneLauncherFunctions droneLauncherFunctions;

    private DeliveryState deliveryState = DeliveryState.DELIVERY_START;

    private final int LIFT_HIGH = 750;

    private final int SET_1_HEIGHT = 1000;
    private final int SET_2_HEIGHT = 1500;

    private final int SET_3_HEIGHT = 1800;

    private final int LIFT_LOW = 0;

    private final double DUMP_TIME = 1;

    private final int LIFT_MAX = 1800;
    private final int LIFT_MIN = 0;

    private int leftMotorPosition;
    private int rightMotorPosition;

    private int targetPosition;

    private boolean dumped = false;
    private boolean secondDump = false;
    private boolean isRunToPosition = true;

    private boolean retracting = false;

    ElapsedTime deliveryTimer = new ElapsedTime();
    static DcMotor[] motors;

    static SampleMecanumDrive drive;

     private static final double HARDWARECHECK_DELAY = 1;

    private ElapsedTime hardwareCheckTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        super.runOpMode();

        drive = new SampleMecanumDrive(hardwareMap);

        aprilTagsFunctions = new AprilTagsFunctions(this);
        deliveryFunctions = new DeliveryFunctions(this, true);
        intakeFunctions = new IntakeFunctions(this);
        drivetrainFunctions = new DrivetrainFunctions(this);
        droneLauncherFunctions = new DroneLauncherFunctions(this);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);




        waitForStart();

        targetPosition = LIFT_LOW;
        deliveryFunctions.setSlidesTargetPosition(LIFT_LOW);

        while (opModeIsActive()) {

            if (intakeFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                intakeFunctions.Reinitialize();
            if (deliveryFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                deliveryFunctions.Reinitialize();
            if (drivetrainFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                drivetrainFunctions.Reinitialize();
            if (droneLauncherFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                droneLauncherFunctions.Reinitialize();

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
                if (gamepad1.b)
                    droneLauncherFunctions.ReleaseDrone();
            }

//            if(gamepad1.dpad_down){
//                vel = new Vector2d(vel.getX(), -1);
//            }
//            if(gamepad1.dpad_up){
//                vel = new Vector2d(vel.getX(), 1);
//            }
//            if(gamepad1.dpad_left){
//                vel = new Vector2d(-1, vel.getY());
//            }
//            if(gamepad1.dpad_right){
//                vel = new Vector2d(1, vel.getY());
//            }

//            if(aprilTagsFunctions.DetectAprilTag(aprilTagsFunctions.BLUE_1_TAG)){
//                telemetry.addData("Found", "ID %d (%s)", aprilTagsFunctions.detectedTag.id, aprilTagsFunctions.detectedTag.metadata.name);
//                telemetry.addData("Range",  "%5.1f inches", aprilTagsFunctions.detectedTag.ftcPose.range);
//                telemetry.addData("Bearing","%3.0f degrees", aprilTagsFunctions.detectedTag.ftcPose.bearing);
//                telemetry.addData("Yaw","%3.0f degrees", aprilTagsFunctions.detectedTag.ftcPose.yaw);
//                telemetry.addData("X delta","%3.0f inches", aprilTagsFunctions.detectedTag.ftcPose.x);
//
//                if(gamepad1.right_trigger > 0.025f){
//                    rightTriggerPull = gamepad1.right_trigger;
//
////                    strafeGain *= rightTriggerPull;
////                    forwardGain *= rightTriggerPull;
////                    rotationGain *= rightTriggerPull;
//
//                    double x = STRAFE_GAIN * aprilTagsFunctions.detectedTag.ftcPose.yaw;
//                    double y = -FORWARD_GAIN * aprilTagsFunctions.detectedTag.ftcPose.range;
//
////                    double x = 0.5;
////                    double y = 0.7;
//                    double bearing = -ROTATION_GAIN * aprilTagsFunctions.detectedTag.ftcPose.bearing;
//
//                    telemetry.addData("x: ", x);
//                    telemetry.addData("y: ", y);
//                    telemetry.addData("bearing: ", bearing);
//
//                    drivetrainFunctions.Move((float)x,(float)y,(float)bearing, 1);
//                } else {
//                    controlsRelinquished = false;
//                }
//
//                /*
//                if (currentGamepad1.right_trigger > 0.5) {
//                    y      = SPEED_GAIN * (aprilTagsFunctions.detectedTag.ftcPose.range - DESIRED_DISTANCE_TO_APRIL_TAG_INCHES);
//                    yaw    = -TURN_GAIN * aprilTagsFunctions.detectedTag.ftcPose.yaw;
//                    x      = STRAFE_GAIN * aprilTagsFunctions.detectedTag.ftcPose.x;
//                    isAutoDrivingToAprilTag = true;
//                }
//                 */
//            }else{          //align to point (pose of aprilTag)
//
//            }
//            float[] XYBearing = aprilTagsFunctions.moveToTag(aprilTagsFunctions.BLUE_1_TAG);
//            drivetrainFunctions.Move(XYBearing[0], XYBearing[1], XYBearing[2], 1);

            if (aprilTagsFunctions.DetectAprilTag(aprilTagsFunctions.BLUE_1_TAG)) {
                telemetry.addData("Found", "ID %d (%s)", aprilTagsFunctions.detectedTag.id, aprilTagsFunctions.detectedTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", aprilTagsFunctions.detectedTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", aprilTagsFunctions.detectedTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", aprilTagsFunctions.detectedTag.ftcPose.yaw);
                telemetry.addData("X delta", "%3.0f inches", aprilTagsFunctions.detectedTag.ftcPose.x);

                if (gamepad1.right_trigger > 0.025f) {
                    rightTriggerPull = gamepad1.right_trigger;

//                    strafeGain *= rightTriggerPull;
//                    forwardGain *= rightTriggerPull;
//                    rotationGain *= rightTriggerPull;

                    double x = STRAFE_GAIN * aprilTagsFunctions.detectedTag.ftcPose.yaw;
                    double y = -FORWARD_GAIN * aprilTagsFunctions.detectedTag.ftcPose.range;

//                    double x = 0.5;
//                    double y = 0.7;
                    double bearing = -ROTATION_GAIN * aprilTagsFunctions.detectedTag.ftcPose.bearing;

                    telemetry.addData("x: ", x);
                    telemetry.addData("y: ", y);
                    telemetry.addData("bearing: ", bearing);

                    drivetrainFunctions.Move((float) x, (float) y, (float) bearing, 1);
                } else {
                    controlsRelinquished = false;
                }

                /*
                if (currentGamepad1.right_trigger > 0.5) {
                    y      = SPEED_GAIN * (aprilTagsFunctions.detectedTag.ftcPose.range - DESIRED_DISTANCE_TO_APRIL_TAG_INCHES);
                    yaw    = -TURN_GAIN * aprilTagsFunctions.detectedTag.ftcPose.yaw;
                    x      = STRAFE_GAIN * aprilTagsFunctions.detectedTag.ftcPose.x;
                    isAutoDrivingToAprilTag = true;
                }
                 */
            } else {          //align to point (pose of aprilTag)

            }

//            if (aprilTagsFunctions.numberOfDetections()>1) {
//                Pose2d aprilTagLocation = aprilTagsFunctions.AverageAbsolutePositionFromAprilTags();
//                telemetry.addLine(String.format("XYH %3.1f %3.1f %3.1f  (in, in, deg)",
//                        aprilTagLocation.getX(),
//                        aprilTagLocation.getY(),
//                        Math.toDegrees(aprilTagLocation.getHeading())));
//
//            }


//            if(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
//                applyFieldOrientedVectorsToPower();
//            }

            telemetry.update();


            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                speedScalar = 0.5;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            } else {
                speedScalar = 1;
            }


            if (gamepad1.y) {
                drivetrainFunctions.ResetIMU();
            }

            if (gamepad1.b) {
                droneLauncherFunctions.ReleaseDrone();
            }

            //Gamepad 2

            //telemetry.addData("RunMode: ", deliveryFunctions.getRunMode());
            targetPosition = deliveryFunctions.getMotorTargetPosition();

            leftMotorPosition = deliveryFunctions.getMotorPositionByIndex(0);
            rightMotorPosition = deliveryFunctions.getMotorPositionByIndex(1);

            //deliveryFunctions.setSlidesPower(0.75);
            //P controlling power at the bottom

            switch (deliveryState) {
                case DELIVERY_START:
                    if (gamepad2.a) {
                        deliveryState = DeliveryState.DELIVERY_LIFT;
                        targetPosition = SET_1_HEIGHT;
                    }
                    if (gamepad2.x) {
                        deliveryState = DeliveryState.DELIVERY_LIFT;
                        targetPosition = SET_2_HEIGHT;
                    }
                    if (gamepad2.y) {
                        deliveryState = DeliveryState.DELIVERY_LIFT;
                        targetPosition = SET_3_HEIGHT;
                    }

                    if (gamepad2.b && !dumped && deliveryFunctions.getMotorPositionByIndex(0) > deliveryFunctions.CARRIAGE_OUTSIDE_CHASSIS) {
                        deliveryState = DeliveryState.DELIVERY_DUMP;
                        dumped = true;
                        deliveryTimer.reset();
                        deliveryFunctions.Dump(1);
                    }
                    break;

                case DELIVERY_LIFT:

                    //if both motors are within stop threshold
                    if
                    (targetPosition - leftMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD
                            &&
                            targetPosition - rightMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD) {
                        deliveryState = DeliveryState.DELIVERY_DUMP;
                    } else {
                        //Still going
                    }
                    break;


                case DELIVERY_DUMP:
                    boolean queuedB = false;
                    if (gamepad2.b) {
                        queuedB = true;
                    }

                    if ((gamepad2.b || queuedB) && !dumped) {
                        queuedB = false;
                        dumped = true;
                        deliveryTimer.reset();
                        deliveryFunctions.Dump(1);
                    }

                    if (dumped && deliveryTimer.seconds() >= DUMP_TIME && (gamepad2.b || queuedB) && !secondDump) {
                        queuedB = false;
                        secondDump = true;
                        deliveryTimer.reset();
                        deliveryFunctions.Dump(2);
                    }

                    if (dumped && deliveryTimer.seconds() >= DUMP_TIME && (gamepad2.b || queuedB) && secondDump) {
                        queuedB = false;
                        dumped = false;
                        secondDump = false;
                        deliveryTimer.reset();
                        retracting = true;
                        deliveryState = DeliveryState.DELIVERY_RETRACT;
                        deliveryFunctions.SetWristPosition(deliveryFunctions.CARRIAGE_DODGE);
                    }

                    break;
                case DELIVERY_RETRACT:
                    retracting = true;
                    if(deliveryTimer.seconds() <= DUMP_TIME + 2){
                        targetPosition = LIFT_LOW;
                        deliveryFunctions.SetWristPosition(deliveryFunctions.CARRIAGE_DODGE);
                        break;
                    }

                    if (deliveryFunctions.getMotorPositionByIndex(0) < deliveryFunctions.CARRIAGE_DODGE) {
                        deliveryFunctions.SetWristPosition(deliveryFunctions.servoIn);
                    } else {
                        deliveryFunctions.SetWristPosition(deliveryFunctions.CARRIAGE_DODGE);
                    }

                    //if both motors are within stop threshold
                    if
                    (targetPosition + leftMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD
                            &&
                            targetPosition + rightMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD) {

                        deliveryState = DeliveryState.DELIVERY_START;
                        retracting = false;
                    } else {
                        //Still Moving
                    }
                    break;
            }

//            telemetry.addData("Delivery State: ", deliveryState);

            if (Math.abs(gamepad2.right_stick_y) >= DEADZONE || Math.abs(gamepad2.left_stick_y) >= DEADZONE) {
                deliveryState = DeliveryState.DELIVERY_START;

//                telemetry.addLine("manual control");

                if (leftMotorPosition > deliveryFunctions.CARRIAGE_OUTSIDE_CHASSIS) {
                    targetPosition -= gamepad2.left_stick_y * 10;
                } else {
                    targetPosition -= gamepad2.left_stick_y * 8;
                }
            }

            targetPosition = Math.max(LIFT_MIN, Math.min(LIFT_MAX, targetPosition));
            deliveryFunctions.setSlidesTargetPosition(targetPosition);

            deliveryFunctions.PControlPower();

            if (!retracting) {
                deliveryFunctions.WristMovementByLiftPosition();
            }


            //telemetry.addData("Target Position: ", targetPosition);
            telemetry.addData("Target Position in DeliveryFunctions: ", deliveryFunctions.getMotorTargetPosition());
            telemetry.addData("Left Motor Position: ", leftMotorPosition);
            telemetry.addData("Right Motor Position: ", rightMotorPosition);

            if (gamepad2.left_bumper) {
                intakeFunctions.OutakeFromIntake(-1f);
//                deliveryFunctions.OpenHolderServoByIndex(0);

            } else if (gamepad2.left_trigger >= 0.05) {
                intakeFunctions.RunIntakeMotor(gamepad2.left_trigger);
                deliveryFunctions.OpenHolderServoByIndex(0);
                deliveryFunctions.OpenHolderServoByIndex(1);
            } else {
                intakeFunctions.StopIntakeMotor();
                deliveryFunctions.CloseHolderServoByIndex(0);
                deliveryFunctions.CloseHolderServoByIndex(1);
            }

            if (gamepad2.right_bumper){
                deliveryFunctions.OpenHolderServoByIndex(0);
                deliveryFunctions.OpenHolderServoByIndex(1);
            }

        }
        //when opmode is NOT active
        safeStop();

    }





    private void safeStop(){
        drivetrainFunctions.Stop();
        deliveryFunctions.setSlidesPower(0);
        intakeFunctions.RunIntakeMotor(0);
    }

}
