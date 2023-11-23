package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
public class DrivetrainFunctions {
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    public IMU imu;
    private LinearOpMode linearOpMode;
    private double CLICKS_PER_METER = 2492.788;

    public DrivetrainFunctions(LinearOpMode l, boolean useIMU)
    {
        linearOpMode = l;
        Initialize();
        if(useIMU)
            InitIMU(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }


    private void Initialize(){
        leftFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "leftFrontAndRightEncoder");
        leftBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "leftRearAndLeftEncoder");
        rightFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "rightFrontAndFrontEncoder");
        rightBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "rightRear");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    private void InitIMU(RevHubOrientationOnRobot.LogoFacingDirection logoOrientation, RevHubOrientationOnRobot.UsbFacingDirection usbOrientation){
        imu = linearOpMode.hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoOrientation,
                usbOrientation));
        imu.initialize(parameters);
    }

    public void Move(double y, double x, double rx, double speedScalar){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFrontDrive.setPower(((y + x + rx) / denominator) * speedScalar);
        leftBackDrive.setPower(((y - x + rx) / denominator) * speedScalar);
        rightFrontDrive.setPower(((y - x - rx) / denominator) * speedScalar);
        rightBackDrive.setPower(((y + x - rx) / denominator) * speedScalar);
    }

    public void MoveFieldOriented (double y, double x, double rx, double speedScalar){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFrontDrive.setPower ((rotY + rotX + rx) / denominator * speedScalar);
        leftBackDrive.setPower((rotY - rotX + rx) / denominator * speedScalar);
        rightFrontDrive.setPower((rotY - rotX - rx) / denominator * speedScalar);
        rightBackDrive.setPower((rotY + rotX - rx) / denominator * speedScalar);
    }

    public boolean areMotorsOn() {
        return rightFrontDrive.getPower()!=0
                || leftFrontDrive.getPower()!=0
                || rightBackDrive.getPower()!=0
                || leftBackDrive.getPower()!=0;
    }

}
