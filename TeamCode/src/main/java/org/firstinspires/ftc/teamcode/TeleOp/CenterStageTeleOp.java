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
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.IntakeFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.Vision.AprilTagsFunctions;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.Arrays;
import java.util.Vector;


@TeleOp(name="CenterStage Teleop", group="A")


public class CenterStageTeleOp extends RoboMom {

    double deadZone = 0.05;

    double speedScalar = 1;


    private float TARGET_DISTANCE_TO_TAG = 12;

    final double STRAFE_GAIN = 0.015;
    final double FORWARD_GAIN = 0.012;

    final double ROTATION_GAIN = 0.017;

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

    private DeliveryState deliveryState = DeliveryState.DELIVERY_START;

    private final int LIFT_HIGH = 500;

    private final int LIFT_LOW = 0;

    private final double DUMP_TIME = 1;

    private final int LIFT_MAX = 1333;
    private final int LIFT_MIN = 0;

    private int leftMotorPosition;
    private int rightMotorPosition;

    private int targetPosition;

    private boolean dumped = false;

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

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        motors = new DcMotor[] {leftFrontDrive, leftBackDrive, rightBackDrive, rightFrontDrive};

        waitForStart();

        targetPosition = LIFT_LOW;
        deliveryFunctions.setSlidesTargetPosition(LIFT_LOW);

        while (opModeIsActive()) {

            if(intakeFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                intakeFunctions.Reinitialize();
            if(deliveryFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                deliveryFunctions.Reinitialize();
            if(drivetrainFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                drivetrainFunctions.Reinitialize();

            if(hardwareCheckTimer.seconds() >= HARDWARECHECK_DELAY)
                hardwareCheckTimer.reset();

            /**GAMEPAD 1**/

            //Store inputted desired velocity (left_stick_x, left_stick_y)
            if(!controlsRelinquished){

                if(Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone){
                    drivetrainFunctions.Move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, speedScalar);
                } else{
                    drivetrainFunctions.Stop();
                }
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

            if(aprilTagsFunctions.DetectAprilTag(aprilTagsFunctions.BLUE_1_TAG)){
                telemetry.addData("Found", "ID %d (%s)", aprilTagsFunctions.detectedTag.id, aprilTagsFunctions.detectedTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", aprilTagsFunctions.detectedTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", aprilTagsFunctions.detectedTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", aprilTagsFunctions.detectedTag.ftcPose.yaw);
                telemetry.addData("X delta","%3.0f inches", aprilTagsFunctions.detectedTag.ftcPose.x);

                if(gamepad1.right_trigger > 0.025f){
                    rightTriggerPull = gamepad1.right_trigger;

//                    strafeGain *= rightTriggerPull;
//                    forwardGain *= rightTriggerPull;
//                    rotationGain *= rightTriggerPull;

                    double x = STRAFE_GAIN * aprilTagsFunctions.detectedTag.ftcPose.yaw;
                    double y = FORWARD_GAIN * aprilTagsFunctions.detectedTag.ftcPose.range;

                    telemetry.addData("x: ", x);
                    telemetry.addData("y: ", y);

//                    double x = 0.5;
//                    double y = 0.7;
                    double bearing = -ROTATION_GAIN * aprilTagsFunctions.detectedTag.ftcPose.bearing;
                    drivetrainFunctions.Move((float)x,(float)y,(float)bearing, speedScalar);
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
            }


//            if(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
//                applyFieldOrientedVectorsToPower();
//            }

            telemetry.update();




            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                speedScalar = 0.5;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            }else {
                speedScalar = 1;
            }


            if (gamepad1.y) {
                drivetrainFunctions.ResetIMU();
            }

    //Gamepad 2

            telemetry.addData("RunMode: ", deliveryFunctions.getRunMode());
            targetPosition = deliveryFunctions.getMotorTargetPosition();

            leftMotorPosition = deliveryFunctions.getMotorPositionByIndex(0);
            rightMotorPosition = deliveryFunctions.getMotorPositionByIndex(1);

            deliveryFunctions.setSlidesPower(0.75);

            switch (deliveryState){
                case DELIVERY_START:
                    if(gamepad2.a){
                        deliveryState = DeliveryState.DELIVERY_LIFT;
                        targetPosition = LIFT_HIGH;
                    }
                    break;

                case DELIVERY_LIFT:

                    //if both motors are within stop threshold
                    if
                    (LIFT_HIGH - leftMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD
                    &&
                    LIFT_HIGH - rightMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD)
                    {
                        deliveryState = DeliveryState.DELIVERY_DUMP;
                    } else{
                        //Still going
                    }
                    break;
                case DELIVERY_DUMP:
                    if(gamepad2.right_bumper && !dumped){
                        dumped = true;
                        deliveryTimer.reset();
                        deliveryFunctions.Dump();
                    }

                    if(dumped && deliveryTimer.seconds() >= DUMP_TIME){
                        deliveryState = DeliveryState.DELIVERY_RETRACT;
                        targetPosition = LIFT_LOW;
                    }
                    break;
                case DELIVERY_RETRACT:

                    //if both motors are within stop threshold
                    if
                    (LIFT_LOW - leftMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD
                            &&
                    LIFT_LOW - rightMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD)
                    {
                        deliveryState = DeliveryState.DELIVERY_START;
                    } else{
                        //Still Moving
                    }
                    break;
            }

            if(Math.abs(gamepad2.left_stick_y) >= DEADZONE){
                deliveryState = DeliveryState.DELIVERY_START;

                telemetry.addLine("manual control");
                //targetPosition += gamepad2.left_stick_y;
//                deliveryFunctions.setSlidesPower(1);

                //if position within max and min, allow manual control
                if
                (leftMotorPosition <= LIFT_MAX
                    &&
                rightMotorPosition <= LIFT_MAX
                    &&
                leftMotorPosition > LIFT_MIN
                    &&
                rightMotorPosition > LIFT_MIN)
                {
                    targetPosition -= gamepad2.left_stick_y * 5;
                    telemetry.addLine("within");
                }

                //if position is greater, allow downwards manual control
                if((leftMotorPosition > LIFT_MAX || rightMotorPosition > LIFT_MAX) && gamepad2.left_stick_y <= 0){
                    targetPosition -= gamepad2.left_stick_y * 5;
                }
                //if position is less, allow upwards manual control
                if((leftMotorPosition < LIFT_MIN || rightMotorPosition < LIFT_MIN) && gamepad2.left_stick_y >= 0){
                    targetPosition -= gamepad2.left_stick_y * 5;
                }

            }

            deliveryFunctions.setSlidesTargetPosition(targetPosition);

            deliveryFunctions.WristMovementByLiftPosition();

            //telemetry.addData("Target Position: ", targetPosition);
            telemetry.addData("Target Position in DeliveryFunctions: ", deliveryFunctions.getMotorTargetPosition());
            telemetry.addData("Left Motor Position: ", leftMotorPosition);
            telemetry.addData("Right Motor Position: ", rightMotorPosition);

            if(gamepad2.left_bumper){
                intakeFunctions.RunIntakeMotor(0.75f);
                deliveryFunctions.OpenHolderServoByIndex(0);
                deliveryFunctions.OpenHolderServoByIndex(1);
            } else if(gamepad2.left_trigger >= 0.05) {
                intakeFunctions.RunIntakeMotor(gamepad2.left_trigger);
                deliveryFunctions.OpenHolderServoByIndex(0);
                deliveryFunctions.OpenHolderServoByIndex(1);
            }else{
                intakeFunctions.StopIntakeMotor();
                deliveryFunctions.CloseHolderServoByIndex(0);
                deliveryFunctions.CloseHolderServoByIndex(1);
            }

        }
        //when opmode is NOT active
        safeStop();

    }



    public static double[] getErrorAdjustmentVals() {
        double poseEstimateHeading = drive.getPoseEstimate().getHeading();



        double[] vals = new double[]{0.0, 0.0, 0.0, 0.0};
        for(DcMotor motor : motors){
//            motor
        }


        return vals;
    }

    private static double getClosestMultipleOf90Degrees(double val){
        if(val >= -45 && val < 45) {
            return 0.0;
        }else if(val >= 45 && val < 135){
            return 90.0;
        }else if(val >= 135 && val < 225){
            return 180.0;
        }//else
        return 0.0;

    }



    private void safeStop(){
        drivetrainFunctions.Stop();
        deliveryFunctions.setSlidesPower(0);
        intakeFunctions.RunIntakeMotor(0);
    }

}
