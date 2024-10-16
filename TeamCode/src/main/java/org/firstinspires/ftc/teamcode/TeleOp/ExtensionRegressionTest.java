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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//This program is designed so that you can run a sinusoidal regression on a set of points
// relating a servo rotation to its subsequent linear extension from a linkage


@TeleOp(name="Extension Regression points", group="Z")


public class ExtensionRegressionTest extends LinearOpMode {

    double MAX_EXTENSION = 0.8505;
    double MIN_EXTENSION = 0.5205;
    double extension = 0;

    int dataPoints = 10;
    int iteration = 0;

    double EXTENSION_RANGE = MAX_EXTENSION-MIN_EXTENSION;

    double EXTENSION_ITERATION = EXTENSION_RANGE/dataPoints;

    public Servo extensionServo = null;

    ElapsedTime inputBuffer = new ElapsedTime();
    double BUFFER = 1;

    @Override
    public void runOpMode() {
        //super.runOpMode();

        extensionServo = hardwareMap.get(Servo.class, "intakeExtension");


        waitForStart();

        extension = MIN_EXTENSION;

        //Record min extension value from joint to end

        while (opModeIsActive()) {
            /**GAMEPAD 1**/

            //press a to move onto next data point
            //measure extension between each data point and input into scatterplot
            if(gamepad1.a && inputBuffer.seconds() > BUFFER){
                telemetry.addData("Iteration:", iteration);
                telemetry.addData("Extension:", extension);
                iteration++;
                extension += EXTENSION_ITERATION;
                extensionServo.setPosition(extension);
                inputBuffer.reset();
                telemetry.update();
            }

        }

        //run sinusoidal regression on scatterplot to get motion profile
    }

    public static Double solveForServoPosition(double extension) {
        double a = -78.26;
        double b = 43.63;
        double c = 58.52 - extension;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            // No real solutions
            return null;
        } else if (discriminant == 0) {
            // One solution
            double x = -b / (2 * a);
            return x >= 0 ? x : null; // Return if it's positive
        } else {
            // Two solutions
            double sqrtDiscriminant = Math.sqrt(discriminant);
            double x1 = (-b + sqrtDiscriminant) / (2 * a);
            return x1;
        }
    }

}



// extension, distance

//.05205, 59.5
//0.5535, 58.75
//0.5865, 57
//0.6195, 55.5
//0.6525, 54
//0.6855, 51
//0.7185, 50
//0.7515, 48
//0.7845, 43.5
//0.8175, 41.5
//0.8505, 39.5
//0.8835, 39.75
//0.9165, 38.75
//0.9495, 36.5
//0.9825, 36.5
