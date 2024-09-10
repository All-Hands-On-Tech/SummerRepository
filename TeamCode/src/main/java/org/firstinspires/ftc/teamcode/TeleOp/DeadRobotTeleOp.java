package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.RoboMom;

@TeleOp(name="DEADROBOT Teleop", group="Summer")

public class DeadRobotTeleOp extends RoboMom {

    private DrivetrainFunctions drivetrainFunctions;

    @Override
    public void runOpMode() {
        super.runOpMode();

        drivetrainFunctions = new DrivetrainFunctions(this);


        waitForStart();

        while (opModeIsActive()) {
            drivetrainFunctions.Move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1);
        }
    }
}
