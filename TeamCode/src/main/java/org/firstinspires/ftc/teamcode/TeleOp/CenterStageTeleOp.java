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

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.Vision.AprilTagsFunctions;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.Vector;


@TeleOp(name="CenterStage Teleop", group="A")


public class CenterStageTeleOp extends RoboMom {

    double deadZone = 0.05;

    double speedScalar = 1;

    public Vector2d vel = new Vector2d(0, 0);
    public double velMag = vel.distTo(new Vector2d(0, 0));

    public double rotateVel = 0;

    private float TARGET_DISTANCE_TO_TAG = 12;

    double lfPower = 0;
    double lbPower = 0;
    double rfPower = 0;
    double rbPower = 0;

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

    private DeliveryState deliveryState;

    private final int LIFT_HIGH = 500;

    private final int LIFT_LOW = 0;

    private final double DUMP_TIME = 1;

    private final int LIFT_MAX = 700;
    private final int LIFT_MIN = 0;

    private int leftMotorPosition;
    private int rightMotorPosition;

    private int targetPosition;

    private boolean dumped = false;

    ElapsedTime deliveryTimer = new ElapsedTime();

     IMU imu;

    @Override
    public void runOpMode() {
        super.runOpMode();

        aprilTagsFunctions = new AprilTagsFunctions(this);
        deliveryFunctions = new DeliveryFunctions(this);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
// Adjust the orientation parameters to match robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

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

            inputVel(new Vector2d(0, 0));
            /**GAMEPAD 1**/

            //Store inputted desired velocity (left_stick_x, left_stick_y)
            if(!controlsRelinquished){

                if(Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.left_stick_y) > deadZone){
                    inputVel(new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y));
                } else{
                    inputVel(new Vector2d(0, 0));
                }
                //Store inputted desired rotational velocity (right_stick_x)
                if(Math.abs(gamepad1.right_stick_x) > deadZone){
                    rotateVel = gamepad1.right_stick_x;
                }else{
                    rotateVel = 0;
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
                    inputVel(new Vector2d(x,y));
                    rotateVel = bearing;
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

            applyVectorsToPower();

//            if(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
//                applyFieldOrientedVectorsToPower();
//            }

//            telemetry.addData("lf", lfPower);
//            telemetry.addData("lb", lbPower);
//            telemetry.addLine();
//            telemetry.addData("rf", rfPower);
//            telemetry.addData("rb", rbPower);
            telemetry.update();




            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                speedScalar = 0.5;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            }else {
                speedScalar = 1;
            }



            leftFrontDrive.setPower(lfPower);
            leftBackDrive.setPower(lbPower);
            rightFrontDrive.setPower(rfPower);
            rightBackDrive.setPower(rbPower);

            if (gamepad1.options) {
                imu.resetYaw();
            }

    //Gamepad 2

            leftMotorPosition = deliveryFunctions.getMotorPositionByIndex(0);
            rightMotorPosition = deliveryFunctions.getMotorPositionByIndex(1);

            deliveryFunctions.setSlidesPower(1);
            switch (deliveryState){
                case DELIVERY_START:
                    if(gamepad2.a){
                        deliveryState = DeliveryState.DELIVERY_LIFT;
                        deliveryFunctions.setSlidesTargetPosition(LIFT_HIGH);
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
                        deliveryFunctions.PControlPower();
                    }
                    break;
                case DELIVERY_DUMP:
                    if(gamepad2.right_bumper != dumped){
                        dumped = true;
                        deliveryTimer.reset();
                        deliveryFunctions.Dump();
                    }

                    if(dumped && deliveryTimer.seconds() >= DUMP_TIME){
                        deliveryState = DeliveryState.DELIVERY_RETRACT;
                        deliveryFunctions.setSlidesTargetPosition(LIFT_LOW);
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
                        deliveryFunctions.PControlPower();
                    }
                    break;
            }

            if(gamepad2.left_stick_y >= DEADZONE){
                deliveryState = DeliveryState.DELIVERY_START;

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
                    targetPosition += gamepad2.left_stick_y;
                }

                //if position is greater, allow downwards manual control
                if(leftMotorPosition > LIFT_MAX && rightMotorPosition > LIFT_MAX && gamepad2.left_stick_y <= 0){
                    targetPosition += gamepad2.left_stick_y;
                }
                //if position is less, allow upwards manual control
                if(leftMotorPosition < LIFT_MIN && rightMotorPosition < LIFT_MIN && gamepad2.left_stick_y >= 0){
                    targetPosition += gamepad2.left_stick_y;
                }

            }

            deliveryFunctions.WristMovementByLiftPosition();


        }


    }



    public void inputVel(Vector2d inputVector){
        vel = inputVector;
//        velMag = vel.distTo(new Vector2d(0, 0));
//        vel = vel.div(velMag); //normalize
    }
    public Vector2d normalize(Vector2d target){
        double targetMag = target.distTo(new Vector2d(0, 0));
        target = target.div(targetMag); //normalize
        return target;
    }
    public void applyVectorsToPower() {


            lfPower = (vel.getY() + vel.getX() + rotateVel / (Math.abs(vel.getY()) + Math.abs(vel.getX()) + Math.abs(rotateVel))) * speedScalar;
            lbPower = (vel.getY() - vel.getX() + rotateVel / (Math.abs(vel.getY()) + Math.abs(vel.getX()) + Math.abs(rotateVel))) * speedScalar;
            rfPower = (vel.getY() - vel.getX() - rotateVel / (Math.abs(vel.getY()) + Math.abs(vel.getX()) + Math.abs(rotateVel))) * speedScalar;
            rbPower = (vel.getY() + vel.getX() - rotateVel / (Math.abs(vel.getY()) + Math.abs(vel.getX()) + Math.abs(rotateVel))) * speedScalar;
            lfPower = vel.getY() + vel.getX() + rotateVel;
            lbPower = vel.getY() - vel.getX() + rotateVel;
            rfPower = vel.getY() - vel.getX() - rotateVel;
            rbPower = vel.getY() + vel.getX() - rotateVel;
    }
    public void applyFieldOrientedVectorsToPower (){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = vel.getX() * Math.cos(-botHeading) - vel.getY() * Math.sin(-botHeading);
        double rotY = vel.getX() * Math.sin(-botHeading) + vel.getY() * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotateVel), 1);
        lfPower = (rotY + rotX + rotateVel) / denominator * speedScalar;
        lbPower = (rotY - rotX + rotateVel) / denominator * speedScalar;
        rfPower = (rotY - rotX - rotateVel) / denominator * speedScalar;
        rbPower = (rotY + rotX - rotateVel) / denominator * speedScalar;
    }


    public double getCurrentZ() {

        return 0.0;
    }

}
