package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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

        //SAUCY BOI CONFIG
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        //SAUCY BOI CONFIG

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontAndRightEncoder");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRearAndLeftEncoder");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontAndFrontEncoder");
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

        double error;

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

            error = currentClicks - targetClicks;

            power = error/500;

            if(power > 0.2){
                driveInDirection(power, direction);
            }

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
    public Pose2d relativePositionFromAprilTag(AprilTagDetection aprilTag) {
        double range = aprilTag.ftcPose.range;
        double yaw = -Math.toRadians(aprilTag.ftcPose.yaw);
        double bearing = Math.toRadians(aprilTag.ftcPose.bearing);

        double x = range * Math.sin(yaw + bearing);
        double y = -range * Math.cos(yaw + bearing);
        double angle = Math.toDegrees(yaw) + 90;

        return new Pose2d(x, y, Math.toRadians(angle));
    }

    public Pose2d absolutePositionFromAprilTag(AprilTagDetection aprilTag) {
        double tagX = 0;
        double tagY = 0;
        double tagAngle = 0;

        //add stuff here based on tag number
        //Needs to be checked
        switch (aprilTag.id) {
            case 1:
                tagX = -41.40;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 2:
                tagX = -35.40;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 3:
                tagX = -29.40;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 4:
                tagX = 29.48;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 5:
                tagX = 35.48;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 6:
                tagX = 41.48;
                tagY = 62.01;
                tagAngle = -90;
                break;
            case 7:
                tagX = 40.93;
                tagY = -70.58;
                tagAngle = -90;
                break;
            case 8:
                tagX = 35.43;
                tagY = -70.58;
                tagAngle = -90;
                break;
            case 9:
                tagX = -35.51;
                tagY = -70.58;
                tagAngle = -90;
            case 10:
                tagX = -41.01;
                tagY = -70.58;
                tagAngle = -90;
                break;
        }

        double range = aprilTag.ftcPose.range;
        double yaw = -Math.toRadians(aprilTag.ftcPose.yaw);
        double bearing = Math.toRadians(aprilTag.ftcPose.bearing);

        //add stuff here based on camera position relative to center of robot
        double x1 = 2;
        double y1 = 7.5;
        tagX-=x1*Math.cos(yaw)-y1*Math.sin(yaw);
        tagY-=x1*Math.sin(yaw)+y1*Math.cos(yaw);

        double x = tagX + range * Math.sin(yaw + bearing);
        double y = tagY - range * Math.cos(yaw + bearing);
        double angle = Math.toDegrees(yaw) + 90;

        return new Pose2d(x, y, Math.toRadians(angle));
    }

    public boolean areMotorsOn() {
        return rightFrontDrive.getPower()!=0
                || leftFrontDrive.getPower()!=0
                || rightBackDrive.getPower()!=0
                || leftBackDrive.getPower()!=0;
    }

    public RevBlinkinLedDriver.BlinkinPattern setLEDColor (String colorFront, String colorBack) {
        //color 1 is yellow
        //color 2 is green

        if (colorBack == "NONE") {
            if (colorFront == "WHITE") {return RevBlinkinLedDriver.BlinkinPattern.WHITE;}
            if (colorFront == "YELLOW") {return RevBlinkinLedDriver.BlinkinPattern.YELLOW;}
            if (colorFront == "GREEN") {return RevBlinkinLedDriver.BlinkinPattern.GREEN;}
            if (colorFront == "PURPLE") {return RevBlinkinLedDriver.BlinkinPattern.BLUE;}
            if (colorFront == "NONE") {return RevBlinkinLedDriver.BlinkinPattern.BLACK;}
        } else if (colorFront == "NONE") {
            if (colorBack == "WHITE") {return RevBlinkinLedDriver.BlinkinPattern.WHITE;}
            if (colorBack == "YELLOW") {return RevBlinkinLedDriver.BlinkinPattern.YELLOW;}
            if (colorBack == "GREEN") {return RevBlinkinLedDriver.BlinkinPattern.GREEN;}
            if (colorBack == "PURPLE") {return RevBlinkinLedDriver.BlinkinPattern.VIOLET;}
        } else {
            if (colorFront == "WHITE") {return RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY;}
            if (colorFront == "YELLOW") {return RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE;}
            if (colorFront == "GREEN") {return RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;}
            if (colorFront == "PURPLE") {return RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;}
        }
        return RevBlinkinLedDriver.BlinkinPattern.BLACK;
    }

}
