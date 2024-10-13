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

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="RR Masher", group="Linear OpMode")
public class RRTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private ElapsedTime deliveryTimer = new ElapsedTime();

    DrivetrainFunctions drivetrainFunctions = null;
    Delivery delivery = null;
    public enum DeliveryState{
        DELIVERY_START,
        DELIVERY_LIFT,
        DELIVERY_RETRACT
    }
    private MrKrabsTeleOp.DeliveryState deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_START;
    private int slidePosition = 0;

    private int targetPosition = 0;

    private boolean controlsRelinquished = false;
    private final double DRIVE_DEADZONE = 0.05;
    private final double SCORE_SPEED_SCALAR = 0.2;

    private double speedScalar = 1;

    //This is the code to add rr actions
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() {

        delivery = new Delivery(this, false);

        drivetrainFunctions = new DrivetrainFunctions(this);

        waitForStart();

        runtime.reset();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

//        Action trajectoryAction1;
//        trajectoryAction1 = drive.actionBuilder(drive.pose)
//                .splineToSplineHeading(new Pose2d(24,24, Math.PI/2),0)
//                .build();
//
        Action trajectoryAction2;
        trajectoryAction2 = drive.actionBuilder(drive.pose)
//                .splineToSplineHeading(new Pose2d(0,0, 0),0)
                .build();

        Action test;
        test = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            runtime.reset();

            drive.updatePoseEstimate();
            Pose2d currentPose = drive.pose;
            TelemetryPacket packet = new TelemetryPacket();

            // updated based on gamepads

            //driver 1
            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                speedScalar = 0.5;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            } else {
                speedScalar = 1;
            }

            if (!controlsRelinquished) {
                float leftX = gamepad1.left_stick_x;
                float leftY = gamepad1.left_stick_y;
                float rightX = gamepad1.right_stick_x;
                float rightY = gamepad1.right_stick_y;
//                if (Math.abs(leftX) > DRIVE_DEADZONE || Math.abs(leftY) > DRIVE_DEADZONE || Math.abs(rightX) > DRIVE_DEADZONE || Math.abs(rightY) > DRIVE_DEADZONE*2) {
//                    if(Math.abs(rightY) > DRIVE_DEADZONE*2) {
//                        drivetrainFunctions.Move(leftX, rightY, rightX, speedScalar);
//                    }else{
//                        drivetrainFunctions.Move(leftX, leftY, rightX, speedScalar);
//                    }
//                } else {
//                    drivetrainFunctions.Stop();
//                }
                if (Math.abs(leftX) > DRIVE_DEADZONE || Math.abs(leftY) > DRIVE_DEADZONE || Math.abs(rightX) > DRIVE_DEADZONE || Math.abs(rightY) > DRIVE_DEADZONE*2) {
                    if(Math.abs(rightY) > DRIVE_DEADZONE*2) {
                        drivetrainFunctions.MoveFieldOriented(leftY, rightX, rightX, speedScalar, drive.pose.heading.real);
                    }else{
                        drivetrainFunctions.MoveFieldOriented(leftY, leftX, rightX, speedScalar, drive.pose.heading.real);
                    }
                } else {
                    drivetrainFunctions.Stop();
                }
            }

            //run to 0, 0, 0 at tangent of 0
            if(gamepad1.x){
                runningActions.add(test);
            }
            if(gamepad1.a){
                runningActions.add(trajectoryAction2);
            }
            if(gamepad1.b){

            }





            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.milliseconds());
            telemetry.update();
        }
    }
}
