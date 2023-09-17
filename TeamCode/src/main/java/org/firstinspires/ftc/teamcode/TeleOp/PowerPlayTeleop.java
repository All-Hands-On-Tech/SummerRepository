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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.RoboMom;


@TeleOp(name="PowerPlay Teleop", group="Z")


public class PowerPlayTeleop extends RoboMom {

    IntegratingGyroscope gyro;

    double integratedHeading = 0;
    double zero = 0;
    double powerLevel = 1;
    double deadZone = 0.5;

    double ARM_MIN_RANGE = .65;
    double ARM_MAX_RANGE = 0.4;

    public DcMotor armMotor = null;

    public Servo claw = null;

    @Override
    public void runOpMode() {
        super.runOpMode();



        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        double lfPower = 0;
        double lbPower = 0;
        double rfPower = 0;
        double rbPower = 0;

        double lfPowerStrafe = 0;
        double lbPowerStrafe = 0;
        double rfPowerStrafe = 0;
        double rbPowerStrafe = 0;

        double lfPowerForwards = 0;
        double lbPowerForwards = 0;
        double rfPowerForwards = 0;
        double rbPowerForwards = 0;

        double lfPowerRotate = 0;
        double lbPowerRotate = 0;
        double rfPowerRotate = 0;
        double rbPowerRotate = 0;


        boolean straight = false;
        boolean strafe = false;
        boolean rotate = false;
        double amountOfMovements = 0;


        initArm();
        waitForStart();

        while (opModeIsActive()) {
            /**GAMEPAD 1**/
            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                powerLevel = 0.5;
            } else if (gamepad1.right_bumper) {
                powerLevel = 0.8;
            }else {
                powerLevel = 1;
            }

            if(Math.abs(gamepad1.left_stick_x) > deadZone) {
                strafe = true;
                lfPowerStrafe = gamepad1.left_stick_x * powerLevel;
                lbPowerStrafe = -gamepad1.left_stick_x * powerLevel;
                rfPowerStrafe = -gamepad1.left_stick_x * powerLevel;
                rbPowerStrafe = gamepad1.left_stick_x * powerLevel;
            }else{
                strafe = false;
                lfPowerStrafe = 0;
                lbPowerStrafe = 0;
                rfPowerStrafe = 0;
                rbPowerStrafe = 0;
            }

            if(Math.abs(gamepad1.left_stick_y) > deadZone) {
                straight = true;
                lfPowerForwards = gamepad1.left_stick_y * powerLevel;
                lbPowerForwards = gamepad1.left_stick_y * powerLevel;
                rfPowerForwards = gamepad1.left_stick_y * powerLevel;
                rbPowerForwards = gamepad1.left_stick_y * powerLevel;
            }else{
                straight = false;
                lfPowerForwards = 0;
                lbPowerForwards = 0;
                rfPowerForwards = 0;
                rbPowerForwards = 0;
            }

            if(Math.abs(gamepad1.right_stick_x) > deadZone){
                rotate = true;
                lfPowerRotate = gamepad1.right_stick_x * powerLevel;
                lbPowerRotate = gamepad1.right_stick_x * powerLevel;
                rfPowerRotate = -gamepad1.right_stick_x * powerLevel;
                rbPowerRotate = -gamepad1.right_stick_x * powerLevel;
            }else{
                rotate = false;
                lfPowerRotate = 0;
                lbPowerRotate = 0;
                rfPowerRotate = 0;
                rbPowerRotate = 0;
            }

            double temp = 0;

            if(rotate){
                temp += 1;
            }
            if(strafe){
                temp += 1;
            }
            if(straight){
                temp += 1;
            }
            amountOfMovements = temp;

            lfPower = (lfPowerStrafe + lfPowerForwards + lfPowerRotate) / amountOfMovements;
            lbPower = (lbPowerStrafe + lbPowerForwards + lbPowerRotate) / amountOfMovements;
            rfPower = (rfPowerStrafe + rfPowerForwards + rfPowerRotate) / amountOfMovements;
            rbPower = (rbPowerStrafe + rbPowerForwards + rbPowerRotate) / amountOfMovements;

            if(amountOfMovements == 0){
                lfPower = 0;
                lbPower = 0;
                rfPower = 0;
                rbPower = 0;
                driveInDirection(0, "FORWARD");
            }

            leftFrontDrive.setPower(lfPower);
            leftBackDrive.setPower(lbPower);
            rightFrontDrive.setPower(rfPower);
            rightBackDrive.setPower(rbPower);

            if(amountOfMovements == 0){
                lfPower = 0;
                lbPower = 0;
                rfPower = 0;
                rbPower = 0;
                driveInDirection(0, "FORWARD");
            }

            if(gamepad1.y){
                resetZero(0);
            }
            if(gamepad1.a){
                resetZero(180);
            }
            if(gamepad1.x){
                resetZero(90);
            }
            if(gamepad1.b){
                resetZero(-90);
            }

            if(gamepad1.dpad_up){
                rotateToZAbs(0, zero);
            } else if(gamepad1.dpad_left){
                rotateToZAbs(90, zero);
            } else if(gamepad1.dpad_right){
                rotateToZAbs(-90, zero);
            } else if(gamepad1.dpad_down){
                rotateToZAbs(180, zero);
            }

            /**GAMEPAD 2**/


            //Distances have not been learned yet

//            double targetHeight = 0;
//            boolean setHeight = false;
//            if (gamepad2.dpad_down) { //Ground Junction
//                targetHeight=1;
//                setHeight = true;
//                //1
//            } else if (gamepad2.dpad_left) { //Low Junction
//                targetHeight=346;
//                setHeight = true;
//                //346
//            } else if (gamepad2.dpad_right) { //Medium  Junction
//                targetHeight=600;
//                setHeight = true;
//                //600
//            } else if (gamepad2.dpad_up) { //High  Junction
//                targetHeight=854;
//                setHeight = true;
//                //854
//            }
//            //Arm to height code doesn't work currently

//            double currentHeight = getArmHeight();
//            double error = targetHeight + currentHeight;
//
//            if (setHeight==true && Math.abs(error)>10) {
//                telemetry.addData("error: ", error);
//                telemetry.addData("Current Height: ", currentHeight);
//                telemetry.update();
//                armMotor.setPower(error / 100);
//            } else {
//                setHeight = false;
//              }

            double armPower = 0;
            if (Math.abs(gamepad2.left_stick_y) > 0.1){
                armPower = gamepad2.left_stick_y*0.75;
            } else {
                armPower = 0;
            }
            if (Math.abs(gamepad2.right_stick_y) > 0.1){
                armPower = gamepad2.right_stick_y*0.9;
            }
            armMotor.setPower(-armPower);

            if(gamepad2.right_bumper){
                closeClaw();

            }else if (!isClawOpen()){
                openClaw();
            }
            if(gamepad2.left_bumper){
                armStop();
            }
            telemetry.addData("Straight?", straight);
            telemetry.addData("Strafe?", strafe);
            telemetry.addData("Rotate?", rotate);
            telemetry.addData("AmountOfMovements:", amountOfMovements);
            telemetry.update();
        }


    }
    void resetZero(double degreeOffZero){
        zero = getCurrentZ() - degreeOffZero;
    }

    public void initArm() {
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(ARM_MIN_RANGE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void rotateToZAbs(double absTargetAngle, double zero){
        double dividend = 50;
        sleep(20);
        integratedHeading = 0;
        double startAngle = getCurrentZ();
        double error = zero + absTargetAngle - getCurrentZ();

        if(error > 180){
            error -= 360;
        }
        if(error < -180){
            error += 360;
        }

        while(Math.abs(error) > 0.1){
            while (error > 0.1) {
                error = zero + absTargetAngle - getCurrentZ();
                if(error > 180){
                    error -= 360;
                }
                if(error < -180){
                    error += 360;
                }
                double proportionalPower = error / dividend;
                proportionalPower = Math.abs(proportionalPower);

                if(proportionalPower < 0.1){
                    proportionalPower = 0.1;
                }

                //rotate left
                driveInDirection(proportionalPower, "ROTATE_LEFT");


                telemetry.addLine("StartAngle: " + startAngle);
                telemetry.addLine("TargetAngle: " + absTargetAngle);
                telemetry.addLine("CumulativeZ: " + getCurrentZ());
                telemetry.addLine("Error: " + error);
                telemetry.addLine("rotation: counter clockwise");
                telemetry.update();

            }

            while (error < -0.1) {
                error = zero + absTargetAngle - getCurrentZ();
                if(error > 180){
                    error -= 360;
                }
                if(error < -180){
                    error += 360;
                }
                double proportionalPower = error / dividend;
                proportionalPower = Math.abs(proportionalPower);

                if(proportionalPower < 0.1){
                    proportionalPower = 0.1;
                }

                //rotate right
                driveInDirection(proportionalPower, "ROTATE_RIGHT");

                //telemetry
                //            telemetry.addLine("currentZ" + getCurrentZ());
                telemetry.addLine("StartAngle: " + startAngle);
                telemetry.addLine("cumulativeZ" + getCurrentZ());
                telemetry.addLine("Error: " + error);
                telemetry.addLine("targetAngle: " + absTargetAngle);
                telemetry.addLine("rotation: clockwise");
                telemetry.update();
            }


            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);

        }
    }

    public double getCurrentZ() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void closeClaw(){
        claw.setPosition(ARM_MIN_RANGE);

    }

    public void openClaw(){
        claw.setPosition(ARM_MAX_RANGE);

    }

    public boolean isClawOpen() {
        return !(claw.getPosition() > 0.1);

    }

    public void armStop() {
        armMotor.setPower(0);
    }

}
