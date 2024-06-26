package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.RoboMom;

@TeleOp(name="Tank Drive", group="AAA")
public class TankDrive extends LinearOpMode {
    //private DrivetrainFunctions drivetrainFunctions;


    private static final double HARDWARECHECK_DELAY = 1;
    private ElapsedTime hardwareCheckTimer = new ElapsedTime();

    double deadZone = 0.05;

    double speedScalar = 1;

    double scoreSpeedScalar = 0.2;

    public DcMotor rightMotor = null;
    public DcMotor leftMotor = null;

    @Override
    public void runOpMode() {
//        super.runOpMode();

        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");

        waitForStart();

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

            if (Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone || Math.abs(gamepad1.right_stick_y) > deadZone) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
                double rx = gamepad1.right_stick_x;
                if(Math.abs(gamepad1.right_stick_x) > deadZone) {


                    leftMotor.setPower(-(y + rx) * speedScalar);
                    rightMotor.setPower((y - rx) * speedScalar);
                }else{
                    leftMotor.setPower((-y) * speedScalar);
                    rightMotor.setPower((y) * speedScalar);
                }
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }


            if (gamepad1.left_bumper) {
                speedScalar = 0.5;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            } else {
                speedScalar = 1;
            }
        }


    }


}
