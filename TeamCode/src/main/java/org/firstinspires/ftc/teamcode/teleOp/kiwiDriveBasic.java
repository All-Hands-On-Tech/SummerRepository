package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic Kiwi Drive", group="B")
public class kiwiDriveBasic extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            double driveX = gamepad1.left_stick_x;
            double driveY = gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_y;
            motor1.setPower(driveX+turn);
            motor2.setPower(-0.5*driveX + 0.866*driveY + turn);
            motor2.setPower(-0.5*driveX - 0.866*driveY + turn);
        }
    }
}
