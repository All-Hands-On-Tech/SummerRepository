package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;


@Disabled
public class DrivetrainFunctions {
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    RevBlinkinLedDriver blinkinLedDriver;

    public IMU imu;
    private LinearOpMode linearOpMode;
    private double CLICKS_PER_METER = 2492.788;

    public boolean isDisabled = false;

    public boolean odometryIsDisabled = false;

    private double initAttempts = 0;

    public static SampleMecanumDrive drive;

    public DrivetrainFunctions(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
        try {
            InitIMU(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        }catch(NullPointerException e){
            linearOpMode.telemetry.addLine("IMU not found");
        }
    }


    private void Initialize(){
        try {
            leftFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "leftFrontAndRightEncoder");
            leftBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "leftRearAndLeftEncoder");
            rightFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "rightFrontAndFrontEncoder");
            rightBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "rightRear");


//            try{
//                drive = new SampleMecanumDrive(linearOpMode.hardwareMap);
//            }catch(NullPointerException e){
//                odometryIsDisabled = true;
//            }

            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            blinkinLedDriver = linearOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find motors.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void Reinitialize(){
        try {
            leftFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "leftFrontAndRightEncoder");
            leftBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "leftRearAndLeftEncoder");
            rightFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "rightFrontAndFrontEncoder");
            rightBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "rightRear");



            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            blinkinLedDriver = linearOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
            isDisabled = false;
        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find motors.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    private void InitIMU(RevHubOrientationOnRobot.LogoFacingDirection logoOrientation, RevHubOrientationOnRobot.UsbFacingDirection usbOrientation){
        if(!isDisabled)
            return;

        imu = linearOpMode.hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoOrientation,
                usbOrientation));
        imu.initialize(parameters);

    }

    public void Move(float x, float y, float rx, double speedScalar){
        if(isDisabled)
            return;
        y = -y;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFrontDrive.setPower(((y + x + rx) / denominator) * speedScalar);
        leftBackDrive.setPower(((y - x + rx) / denominator) * speedScalar);
        rightFrontDrive.setPower(((y - x - rx) / denominator) * speedScalar);
        rightBackDrive.setPower(((y + x - rx) / denominator) * speedScalar);
    }

    public void MoveFieldOriented (float x, float y, float rx, double speedScalar){
        if(isDisabled)
            return;

        y = -y;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotY = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);
        double rotX = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFrontDrive.setPower((rotX + rotY + rx) / denominator * speedScalar);
        leftBackDrive.setPower((rotX - rotY + rx) / denominator * speedScalar);
        rightFrontDrive.setPower((rotX - rotY - rx) / denominator * speedScalar);
        rightBackDrive.setPower((rotX + rotY - rx) / denominator * speedScalar);
    }

    public void Stop(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void ResetIMU(){
        if(isDisabled)
            return;
        imu.resetYaw();
    }

    public boolean areMotorsOn() {
        if(isDisabled)
            return false;

        return rightFrontDrive.getPower() != 0
                || leftFrontDrive.getPower() != 0
                || rightBackDrive.getPower() != 0
                || leftBackDrive.getPower() != 0;
    }

    public void setLEDColor (String colorFront, String colorBack, String extras) {
        if (extras == "BACK") {
            if (colorBack == "WHITE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);}
            if (colorBack == "YELLOW") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);}
            if (colorBack == "GREEN") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);}
            if (colorBack == "PURPLE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);}
            if (colorBack == "NONE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);}
        }
        if (extras == "PARTY") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);}

        if (colorBack == "NONE") {
            if (colorFront == "WHITE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);}
            if (colorFront == "YELLOW") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);}
            if (colorFront == "GREEN") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);}
            if (colorFront == "PURPLE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);}
            if (colorFront == "NONE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);}
        } else if (colorFront == "NONE") {
            if (colorBack == "WHITE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);}
            if (colorBack == "YELLOW") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);}
            if (colorBack == "GREEN") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);}
            if (colorBack == "PURPLE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);}
        } else {
            if (colorFront == "WHITE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY);}
            if (colorFront == "YELLOW") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE);}
            if (colorFront == "GREEN") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);}
            if (colorFront == "PURPLE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);}
        }
    }




}
