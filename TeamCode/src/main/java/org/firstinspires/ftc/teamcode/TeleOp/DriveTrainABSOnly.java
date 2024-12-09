package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.RoboMom;

@TeleOp(name="DriveTrainOnly", group="TEST")
public class DriveTrainABSOnly extends LinearOpMode {
    private DrivetrainFunctions drivetrainFunctions;

    double deadZone = 0.05;

    double speedScalar = 1;
    @Override
    public void runOpMode() {
        drivetrainFunctions = new DrivetrainFunctions(this);
        waitForStart();
        while (opModeIsActive()) {

            if (Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone || Math.abs(gamepad1.right_stick_y) > deadZone*2) {
                    drivetrainFunctions.MoveABS(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, speedScalar);
            } else {
                drivetrainFunctions.BrakeABS();
            }


            if (gamepad1.left_bumper) {
                speedScalar = 0.25;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            } else {
                speedScalar = 1;
            }
        }
    }
}
