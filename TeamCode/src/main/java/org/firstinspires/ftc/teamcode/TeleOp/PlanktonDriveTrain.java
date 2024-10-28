package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="Plankton DRIVETRAIN ONLY", group="TEST")
public class PlanktonDriveTrain extends LinearOpMode {

    private ElapsedTime deliveryTimer = new ElapsedTime();

    DrivetrainFunctions drivetrainFunctions = null;


    private boolean controlsRelinquished = false;
    private final double DRIVE_DEADZONE = 0.05;
    private final double SCORE_SPEED_SCALAR = 0.2;

    private double speedScalar = 1;


    @Override
    public void runOpMode() {

        drivetrainFunctions = new DrivetrainFunctions(this);


        waitForStart();

        while (opModeIsActive()){

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
                if (Math.abs(leftX) > DRIVE_DEADZONE || Math.abs(leftY) > DRIVE_DEADZONE || Math.abs(rightX) > DRIVE_DEADZONE || Math.abs(rightY) > DRIVE_DEADZONE*2) {
                    if(Math.abs(rightY) > DRIVE_DEADZONE*2) {
                        drivetrainFunctions.Move(leftX, rightY, rightX, speedScalar);
                    }else{
                        drivetrainFunctions.Move(leftX, leftY, rightX, speedScalar);
                    }
                } else {
                    drivetrainFunctions.Stop();
                }
            }

            //driver 2

        }
    }
}
