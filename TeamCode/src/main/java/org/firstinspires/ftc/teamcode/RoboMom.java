package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

public abstract class RoboMom extends LinearOpMode {

    //Initializing hardware variables
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    public void runOpMode() {
        //Defining hardware variables
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    }

    //Function that has the robot drive in one of these directions:
    //FORWARD, BACKWARD, RIGHT, LEFT, ROTATE_RIGHT, ROTATE_LEFT, STOP
    //Uses variable power to set the power to the motors (units arbitrary 0-1)
    public void driveInDirection(double power, String direction){
        switch(direction){
            case "FORWARD":
                rightFrontDrive.setPower(power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(power);
                leftBackDrive.setPower(power);
                break;
            case "BACKWARD":
                rightFrontDrive.setPower(-power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(-power);
                leftBackDrive.setPower(-power);
                break;
            case "RIGHT":
                rightFrontDrive.setPower(-power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(power);
                leftBackDrive.setPower(-power);
                break;
            case "LEFT":
                rightFrontDrive.setPower(power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(-power);
                leftBackDrive.setPower(power);
                break;
            case "ROTATE_RIGHT":
                rightFrontDrive.setPower(-power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(-power);
                leftBackDrive.setPower(power);
                break;
            case "ROTATE_LEFT":
                rightFrontDrive.setPower(power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(power);
                leftBackDrive.setPower(-power);
                break;
            case "STOP":
                rightFrontDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);
                break;
        }
    }

    //Function uses the motor encoders to estimate distance traveled (units centimeters)
    //Uses driveInDirection for power and direction
    public void driveForDistance(double power, double distance, String direction){
        double clicksPerMeter = 2492.788;
        double targetClicks = distance*clicksPerMeter;
        double currentClicks = 0;

        double rightFrontClicks;
        double leftFrontClicks;
        double rightBackClicks;
        double leftBackClicks;

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveInDirection(power, direction);

        while(currentClicks < targetClicks){

            rightFrontClicks = Math.abs(rightFrontDrive.getCurrentPosition());
            leftFrontClicks = Math.abs(leftFrontDrive.getCurrentPosition());
            rightBackClicks = Math.abs(rightBackDrive.getCurrentPosition());
            leftBackClicks = Math.abs(leftBackDrive.getCurrentPosition());

            currentClicks = (rightFrontClicks+leftFrontClicks+rightBackClicks+leftBackClicks)/4;
        }
        driveInDirection(0, "FORWARD");
    }

    //Function uses the internal to tell time elapsed (units seconds)
    //Uses driveInDirection for power and direction
    public void driveForTime(String direction, double power, double time){
        time*=1000;
        driveInDirection(power, direction);
        sleep((long)time);
        driveInDirection(0, "STOP");
    }

    //units are in inches!!!!!!!
    public Pose2d AbsolutePositionFromAprilTag(AprilTagDetection aprilTag) {
        //add stuff here based on tag number
        double tagX = 0;
        double tagY = 0;
        double tagAngle = 0;

        switch (aprilTag.id) {
            case 1:
                tagX = 29.15;
                tagY = 132;
                tagAngle = 0;
                break;
            case 2:
                tagX = 35.15;
                tagY = 132;
                tagAngle = 0;
                break;
            case 3:
                tagX = 41.15;
                tagY = 132;
                tagAngle = 0;
                break;
            case 4:
                tagX = 100.05;
                tagY = 132;
                tagAngle = 0;
                break;
            case 5:
                tagX = 106.00;
                tagY = 132;
                tagAngle = 0;
                break;
            case 6:
                tagX = 112.00;
                tagY = 132;
                tagAngle = 0;
                break;
            case 7:
                tagX = -111.34;
                tagY = 0;
                tagAngle = 180;
                break;
            case 8:
                tagX = 105.86;
                tagY = 0;
                tagAngle = 180;
                break;
            case 9:
                tagX = 35;
                tagY = 0;
                tagAngle = 180;
                break;
            case 10:
                tagX = 29.5;
                tagY = 0;
                tagAngle = 180;
                break;
        }

        double range = aprilTag.ftcPose.range;
        double yaw = aprilTag.ftcPose.yaw;
        double bearing = aprilTag.ftcPose.bearing;

        double rightAngle = Math.PI/4;

        double x = tagX + range * Math.cos(yaw + bearing - rightAngle);
        double y = tagY + range * Math.sin( yaw + bearing - rightAngle);
        double angle = tagAngle - yaw;

        return new Pose2d(range, yaw, bearing);


    }


}
