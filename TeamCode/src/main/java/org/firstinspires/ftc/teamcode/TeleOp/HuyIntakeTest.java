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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoboMom;


@TeleOp(name="IntakeTest", group="Z")


public class HuyIntakeTest extends LinearOpMode {
    double deadZone = 0.5;

    double MAX_EXTENSION =1;
    double MIN_EXTENSION = 0;
    double extension = 0;

    public DcMotor pitchMotor = null;

    public Servo extensionServo = null;
    public Servo intakeServo = null;

    @Override
    public void runOpMode() {
        //super.runOpMode();

        pitchMotor = hardwareMap.get(DcMotor.class, "intakePitch");
        extensionServo = hardwareMap.get(Servo.class, "intakeExtension");
        intakeServo = hardwareMap.get(Servo.class, "intakeEndEffector");


        waitForStart();

        while (opModeIsActive()) {
            /**GAMEPAD 1**/

            double rightY = gamepad1.right_stick_y;


            if (Math.abs(gamepad1.left_stick_y) > deadZone) {
                pitchMotor.setPower(gamepad1.left_stick_y/2);
            } else {
                pitchMotor.setPower(0);
            }

            if (Math.abs(rightY) > deadZone) {
                extension += rightY/10;
                extension = Math.max(39.5, Math.min(59, extension));
                double servoPosition = solveForServoPosition(extension);
                if(servoPosition == 0){break;}
                extensionServo.setPosition(servoPosition);
                telemetry.addData("extension:",extension);
                telemetry.update();
            }

            if(gamepad1.right_bumper){
                intakeServo.setPosition(1);
            }

            if(gamepad1.left_bumper){
                intakeServo.setPosition(0);
            }

            if(!gamepad1.left_bumper && !gamepad1.right_bumper){
                intakeServo.setPosition(0.5);
            }
            }
    }

    public static Double solveForServoPosition(double extension) {
        double a = -78.26;
        double b = 43.63;
        double c = 58.52 - extension;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            // No real solutions
            return 0.0;
        } else if (discriminant == 0) {
            // One solution
            double x = -b / (2 * a);
            return x >= 0 ? x : null; // Return if it's positive
        } else {
            // Two solutions
            double sqrtDiscriminant = Math.sqrt(discriminant);
            double x1 = (-b - sqrtDiscriminant) / (2 * a);
            return x1;
        }
    }
}
