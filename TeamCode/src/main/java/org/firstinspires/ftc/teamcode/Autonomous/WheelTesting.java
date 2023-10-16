package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoboMom;

@Autonomous
public class WheelTesting extends RoboMom {

@Override
    public void runOpMode() {
        super.runOpMode();

        double rFClicks = 0;
        double lFClicks = 0;
        double rBClicks = 0;
        double lBClicks = 0;


        double targetClicks = 1000;
        double currentClicks = 0;

    rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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
            rightFrontDrive.setPower(1);
            leftFrontDrive.setPower(1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(1);

            while(currentClicks < targetClicks){

                rFClicks = Math.abs(rightFrontDrive.getCurrentPosition());
                lFClicks = Math.abs(leftFrontDrive.getCurrentPosition());
                rBClicks = Math.abs(rightBackDrive.getCurrentPosition());
                lBClicks = Math.abs(leftBackDrive.getCurrentPosition());

                currentClicks = (rFClicks+lFClicks+rBClicks+lBClicks)/4;
            }

            driveInDirection(0, "FORWARD");


            telemetry.addData("rF enc dist: ", rFClicks);
            telemetry.addData("lF enc dist: ", lFClicks);
            telemetry.addData("rB enc dist: ", rBClicks);
            telemetry.addData("lB enc dist: ", lBClicks);
            telemetry.update();

            sleep(100000);

            return;

        }

    }
}
