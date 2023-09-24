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


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoboMom;
import com.qualcomm.robotcore.hardware.IMU;


import java.util.Vector;



@TeleOp(name="CenterStage TeleOp", group="A")


public class CenterStageTeleOp extends RoboMom {

    double deadZone = 0.05;

    double speedScalar = 1;

    public Vector2d vel = new Vector2d(0, 0);
    public double velMag = vel.distTo(new Vector2d(0, 0));

    public double rotateVel = 0;

    double lfPower = 0;
    double lbPower = 0;
    double rfPower = 0;
    double rbPower = 0;

     IMU imu;

    @Override
    public void runOpMode() {
        super.runOpMode();

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
// Adjust the orientation parameters to match your robot
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
            /**GAMEPAD 1**/

            //Store inputted desired velocity (left_stick_x, left_stick_y)
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

            applyVectorsToPower();
            if(gamepad1.dpad_down || gamepad1.dpad_down || gamepad1.dpad_down) {
                applyFieldOrientedVectorsToPower();
            }

            telemetry.addData("lf", lfPower);
            telemetry.addData("lb", lbPower);
            telemetry.addLine();
            telemetry.addData("rf", rfPower);
            telemetry.addData("rb", rbPower);
            telemetry.update();




            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                speedScalar = 0.5;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            }else {
                speedScalar = 1;
            }

            if(Math.abs(gamepad1.left_stick_x) > deadZone) {

            }else{

            }

            if(Math.abs(gamepad1.left_stick_y) > deadZone) {

            }else{

            }

            if(Math.abs(gamepad1.right_stick_x) > deadZone){

            }else{

            }





//            lfPower = (lfPowerStrafe + lfPowerForwards + lfPowerRotate) / amountOfMovements;
//            lbPower = (lbPowerStrafe + lbPowerForwards + lbPowerRotate) / amountOfMovements;
//            rfPower = (rfPowerStrafe + rfPowerForwards + rfPowerRotate) / amountOfMovements;
//            rbPower = (rbPowerStrafe + rbPowerForwards + rbPowerRotate) / amountOfMovements;



            leftFrontDrive.setPower(lfPower);
            leftBackDrive.setPower(lbPower);
            rightFrontDrive.setPower(rfPower);
            rightBackDrive.setPower(rbPower);

            if (gamepad1.options) {
                imu.resetYaw();
            }


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
        boolean rotating;
        boolean straight;
        boolean strafe;
        if(Math.abs(gamepad1.right_stick_x) > deadZone){
            rotating = true;
        }else{rotating = false;}
        if(Math.abs(gamepad1.left_stick_y) > deadZone){
            straight = true;
        }else{straight = false;}
        if(Math.abs(gamepad1.left_stick_x) > deadZone){
            strafe = true;
        }else{strafe = false;}

        if(rotating && straight && strafe) {

            lfPower = (vel.getY() + vel.getX() + rotateVel / (Math.abs(vel.getY()) + Math.abs(vel.getX()) + Math.abs(rotateVel))) * speedScalar;
            lbPower = (vel.getY() - vel.getX() + rotateVel / (Math.abs(vel.getY()) + Math.abs(vel.getX()) + Math.abs(rotateVel))) * speedScalar;
            rfPower = (vel.getY() - vel.getX() - rotateVel / (Math.abs(vel.getY()) + Math.abs(vel.getX()) + Math.abs(rotateVel))) * speedScalar;
            rbPower = (vel.getY() + vel.getX() - rotateVel / (Math.abs(vel.getY()) + Math.abs(vel.getX()) + Math.abs(rotateVel))) * speedScalar;

        }
//        if(rotating && !straight && !strafe){
//            lfPower = rotateVel * speedScalar;
//            lbPower = rotateVel * speedScalar;
//            rfPower = -rotateVel * speedScalar;
//            rbPower = -rotateVel * speedScalar;
//        }
//
//        if(straight && !rotating && !strafe) {
//
//            lfPower = vel.getY() * speedScalar;
//            lbPower = vel.getY() * speedScalar;
//            rfPower = vel.getY() * speedScalar;
//            rbPower = vel.getY() * speedScalar;
//
//        }
//        if(strafe && !straight && ! rotating) {
//
//            lfPower = + vel.getX() * speedScalar;
//            lbPower = - vel.getX() * speedScalar;
//            rfPower = - vel.getX() * speedScalar;
//            rbPower = + vel.getX() * speedScalar;
//
//        }
//
//        if(rotating && straight && !strafe) {
//
//            lfPower = (vel.getY() + rotateVel / 3) * speedScalar;
//            lbPower = (vel.getY() + rotateVel / 3) * speedScalar;
//            rfPower = (vel.getY() - rotateVel / 3) * speedScalar;
//            rbPower = (vel.getY() - rotateVel / 3) * speedScalar;
//
//        }
//
//        if(rotating && !straight && strafe) {
//
//            lfPower = (+ vel.getX() + rotateVel / 2) * speedScalar;
//            lbPower = (- vel.getX() + rotateVel / 2) * speedScalar;
//            rfPower = (- vel.getX() - rotateVel / 2) * speedScalar;
//            rbPower = (+ vel.getX() - rotateVel / 2) * speedScalar;
//
//        }
//
//        if(!rotating && straight && strafe) {
//
//            lfPower = (vel.getY() + vel.getX() / 2) * speedScalar;
//            lbPower = (vel.getY() - vel.getX() / 2) * speedScalar;
//            rfPower = (vel.getY() - vel.getX() / 2) * speedScalar;
//            rbPower = (vel.getY() + vel.getX() / 2) * speedScalar;
//
//        }

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
