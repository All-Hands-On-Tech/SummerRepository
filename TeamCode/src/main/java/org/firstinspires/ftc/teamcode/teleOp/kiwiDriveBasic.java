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
            double turn  =  gamepad1.right_stick_x * -0.3;

            double power1 = driveX+turn;
            double power2 = -0.5*driveX + 0.866*driveY + turn;
            double power3 = -0.5*driveX - 0.866*driveY + turn;
            double max = Math.max(Math.abs(power1), Math.max(Math.abs(power2), Math.abs(power3)));

            if (max<1) {max = 1;}

            telemetry.addData("Power 1: ", power1);
            telemetry.addData("Power 2: ", power2);
            telemetry.addData("Power 3: ", power3);

            motor1.setPower(power1/max);
            motor2.setPower(power2/max);
            motor3.setPower(power3/max);
        }
    }
}
