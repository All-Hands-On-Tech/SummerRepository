package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Disabled
public class DrivetrainFunctions {
    public DcMotor leftFrontDrive = null;
    public double leftFrontPower = 0;

    public DcMotor leftBackDrive = null;
    public double leftBackPower = 0;

    public DcMotor rightFrontDrive = null;
    public double rightFrontPower = 0;

    public DcMotor rightBackDrive = null;
    public double rightBackPower = 0;


    public IMU imu;
    private LinearOpMode linearOpMode;
    private double CLICKS_PER_METER = 2492.788;

    public boolean isDisabled = false;

    public boolean odometryIsDisabled = false;

    private double initAttempts = 0;

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

            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

    public void setLeftFrontPower(double power) {
        if (Math.abs(power-leftFrontPower) > 0.02) {
            leftFrontPower = power;
            leftFrontDrive.setPower(power);
        }
    }
    public void setLeftBackPower(double power) {
        if (Math.abs(power-leftBackPower) > 0.02) {
            leftBackPower = power;
            leftBackDrive.setPower(power);
        }
    }
    public void setRightFrontPower(double power) {
        if (Math.abs(power-rightFrontPower) > 0.02) {
            rightFrontPower = power;
            rightFrontDrive.setPower(power);
        }
    }
    public void setRightBackPower(double power) {
        if (Math.abs(power-rightBackPower) > 0.02) {
            rightBackPower = power;
            rightBackDrive.setPower(power);
        }
    }

    public void Move(float y, float x, float rx, double speedScalar){ //x is forward/backward
        if(isDisabled)
            return;
        x = -x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        setLeftFrontPower(((-y + x + rx) / denominator) * speedScalar);
        setLeftBackPower(((y + x + rx) / denominator) * speedScalar);
        setRightFrontPower(((-y + x - rx) / denominator) * speedScalar);
        setRightBackPower(((y + x - rx) / denominator) * speedScalar);
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
        setLeftFrontPower((rotX + rotY + rx) / denominator * speedScalar);
        setLeftBackPower((rotX - rotY + rx) / denominator * speedScalar);
        setRightFrontPower((rotX - rotY - rx) / denominator * speedScalar);
        setRightBackPower((rotX + rotY - rx) / denominator * speedScalar);
    }

    public void Stop(){
        leftFrontDrive.setPower(0);
        leftFrontPower = 0;

        leftBackDrive.setPower(0);
        leftBackPower = 0;

        rightFrontDrive.setPower(0);
        rightFrontPower = 0;

        rightBackDrive.setPower(0);
        rightBackPower = 0;
    }

    public void ResetIMU(){
        if(isDisabled)
            return;
        imu.resetYaw();
    }

    public boolean areMotorsOn() {
        if(isDisabled)
            return false;

        return rightFrontPower != 0
                || leftFrontPower != 0
                || rightBackPower != 0
                || leftBackPower != 0;
    }




}
