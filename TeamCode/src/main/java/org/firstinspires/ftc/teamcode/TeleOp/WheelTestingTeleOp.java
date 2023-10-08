package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoboMom;

@TeleOp
public class WheelTestingTeleOp extends RoboMom {

@Override
    public void runOpMode() {
        super.runOpMode();
        double rFClicks = 0;
        double lFClicks = 0;
        double rBClicks = 0;
        double lBClicks = 0;


        double targetClicks = 1000;
        double currentClicks = 0;



        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.a){
                rightFrontDrive.setPower(1);
                leftFrontDrive.setPower(1);
                rightBackDrive.setPower(1);
                leftBackDrive.setPower(1);
            }else{
                rightFrontDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);
            }
            rFClicks = Math.abs(rightFrontDrive.getCurrentPosition());
            lFClicks = Math.abs(leftFrontDrive.getCurrentPosition());
            rBClicks = Math.abs(rightBackDrive.getCurrentPosition());
            lBClicks = Math.abs(leftBackDrive.getCurrentPosition());



            telemetry.addData("rF enc dist: ", rFClicks);
            telemetry.addData("lF enc dist: ", lFClicks);
            telemetry.addData("rB enc dist: ", rBClicks);
            telemetry.addData("lB enc dist: ", lBClicks);
            telemetry.update();

        }

    }
}
