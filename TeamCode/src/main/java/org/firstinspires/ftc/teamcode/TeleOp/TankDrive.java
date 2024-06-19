package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.RoboMom;

@TeleOp(name="DriveTrainOnly", group="AAA")
public class TankDrive extends RoboMom {
    private DrivetrainFunctions drivetrainFunctions;


    private static final double HARDWARECHECK_DELAY = 1;
    private ElapsedTime hardwareCheckTimer = new ElapsedTime();

    double deadZone = 0.05;

    double speedScalar = 1;

    double scoreSpeedScalar = 0.2;

    @Override
    public void runOpMode() {
        super.runOpMode();
        drivetrainFunctions = new DrivetrainFunctions(this);
        waitForStart();
        while (opModeIsActive()) {


            if (drivetrainFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                drivetrainFunctions.Reinitialize();

            if (hardwareCheckTimer.seconds() >= HARDWARECHECK_DELAY)
                hardwareCheckTimer.reset();


            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone || Math.abs(gamepad1.right_stick_y) > deadZone*2) {
                if(Math.abs(gamepad1.right_stick_y) > deadZone*2) {
                    drivetrainFunctions.Move(gamepad1.left_stick_x, gamepad1.right_stick_y, gamepad1.right_stick_x, scoreSpeedScalar);
                }else{
                    drivetrainFunctions.Move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, speedScalar);
                }
            } else {
                drivetrainFunctions.Stop();
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
