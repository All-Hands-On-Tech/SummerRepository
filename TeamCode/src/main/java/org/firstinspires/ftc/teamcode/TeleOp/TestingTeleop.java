/* Copyright (c) 2017 FIRST. All rights reserved.
 * Rohan copyrighted dis
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.DroneLauncherFunctions;
import org.firstinspires.ftc.teamcode.IntakeFunctions;
import org.firstinspires.ftc.teamcode.LEDFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.VisionFunctions;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;


@TeleOp(name="Testing Teleop", group="ZZZ")


public class TestingTeleop extends RoboMom {

    double deadZone = 0.05;

    double speedScalar = 1;

    private double scoreSpeedScalar = 0.2;


    private float TARGET_DISTANCE_TO_TAG = 12;

    final double STRAFE_GAIN = 0.0271;
    final double FORWARD_GAIN = 0.0288;

    final double ROTATION_GAIN = 0.022;

    double rightTriggerPull;

    private VisionFunctions aprilTagsFunctions;

    private boolean controlsRelinquished = false;

    public enum DeliveryState{
        DELIVERY_START,
        DELIVERY_LIFT,
        DELIVERY_DUMP,
        DELIVERY_RETRACT
    }

    private final double DEADZONE = 0.1;

    private DrivetrainFunctions drivetrainFunctions;

    static DcMotor[] motors;

    static SampleMecanumDrive drive;

    private static final double HARDWARECHECK_DELAY = 1;

    private ElapsedTime hardwareCheckTimer = new ElapsedTime();

    StandardTrackingWheelLocalizer myLocalizer;


    @Override
    public void runOpMode() {
        super.runOpMode();

        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        aprilTagsFunctions = new VisionFunctions(this);

        drivetrainFunctions = new DrivetrainFunctions(this);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        aprilTagsFunctions.startDetectingApriltags();


        myLocalizer.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));

        waitForStart();



        String LEDExtras = "";

        while (opModeIsActive()) {
            myLocalizer.update();

            Pose2d poseEstimate = myLocalizer.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            if (hardwareCheckTimer.seconds() >= HARDWARECHECK_DELAY)
                hardwareCheckTimer.reset();

            /**GAMEPAD 1**/

            //Store inputted desired velocity (left_stick_x, left_stick_y)
            if (!controlsRelinquished) {

                if (Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone || Math.abs(gamepad1.right_stick_y) > deadZone*2) {
                    if(Math.abs(gamepad1.right_stick_y) > deadZone*2) {
                        drivetrainFunctions.Move(gamepad1.left_stick_x, gamepad1.right_stick_y, gamepad1.right_stick_x, scoreSpeedScalar);
                    }else{
                        drivetrainFunctions.Move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, speedScalar);
                    }
                } else {
                    drivetrainFunctions.Stop();
                }

            }





                if (gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_right) {

                    controlsRelinquished = true;

                    double x = STRAFE_GAIN * aprilTagsFunctions.detectedTag.ftcPose.yaw;
                    double y = -FORWARD_GAIN * aprilTagsFunctions.detectedTag.ftcPose.range;
                    double bearing = -ROTATION_GAIN * aprilTagsFunctions.detectedTag.ftcPose.bearing;

                    telemetry.addData("x: ", x);
                    telemetry.addData("y: ", y);
                    telemetry.addData("bearing: ", bearing);

                    drivetrainFunctions.Move((float) x, (float) y, (float) bearing, 0.5);
                } else {
                    controlsRelinquished = false;
                }

            if (gamepad1.left_bumper) {
                speedScalar = 0.5;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            } else {
                speedScalar = 1;
            }

            if(gamepad1.start){
                myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
//                myLocalizer.setPoseEstimate(new Pose2d(myLocalizer.getPoseEstimate().getX(), myLocalizer.getPoseEstimate().getY(), 0));
                telemetry.addLine("Reset Pose Estimate");
            }

            if(gamepad1.back){
                Trajectory traj = drive.trajectoryBuilder(myLocalizer.getPoseEstimate())
                        .lineTo(new Vector2d(0, 0))
                        .build();

                drive.followTrajectoryAsync(traj);
            }

            telemetry.update();
            }



    }

}
