package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private LinearOpMode linearOpMode;
    private double CLICKS_PER_METER = 2492.788;

    public boolean isDisabled = false;

    private double initAttempts = 0;

    private ElapsedTime ABSTimer;

    private static final double BRAKE_CYCLE = 10;
    private static final double FLOAT_CYCLE = 25;

    private double brakePowerScalar = 0.9;
    private int brakeCycleNum = 0;
    private static final int TOTAL_BRAKE_CYCLES = 5;

    private double initialStopPower = 1.0;
    private float yIn,  xIn,  rxIn;

    private boolean brakeMode = true;
    private boolean floatMode = false;

    public DrivetrainFunctions(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
        ABSTimer = new ElapsedTime();
    }


    private void Initialize(){
        try {
            leftFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "LF");
            leftBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "LBLE");
            rightFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "RFBE");
            rightBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "RBRE");


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

            //RB, RF, LB, LF

        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find motors.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void Reinitialize(){
        try {
            leftFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "LFRE");
            leftBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "LBLE");
            rightFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "RFBE");
            rightBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "RB");



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

    public double getX(){
        return (leftFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition())/2.0;
    }

    /**
     * This code drives the robot relative to the robot itself
     * @param  x  joy stick y
     * @param  y  joy stick x
     * @param  rx  joy stick rotation
     * @param  speedScalar speed multiplier
     */
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

    /**
     * This code drives the robot relative to is position on the field
     * @param  x  joy stick y (Forward/Backward)
     * @param  y  joy stick x (Left/Right)
     * @param  rx  joy stick rotation
     * @param  speedScalar speed multiplier
     * @param  botHeading  the current heading of the robot (in radians)
     */
    public void MoveFieldOriented (float x, float y, float rx, double speedScalar, double botHeading){
        if(isDisabled)
            return;

        x = -x;

        double rotY = y * Math.cos(-botHeading) + x * Math.sin(-botHeading);
        double rotX = y * Math.sin(-botHeading) - x * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        setLeftFrontPower((rotX + rotY + rx) / denominator * speedScalar);
        setLeftBackPower((rotX - rotY + rx) / denominator * speedScalar);
        setRightFrontPower((rotX - rotY - rx) / denominator * speedScalar);
        setRightBackPower((rotX + rotY - rx) / denominator * speedScalar);
    }

    public void Stop(){
        setLeftFrontPower(0);
        setLeftBackPower(0);
        setRightFrontPower(0);
        setRightBackPower(0);
    }

    public void MoveABS(float y, float x, float rx, double speedScalar){ //x is forward/backward
        if(isDisabled)
            return;
        x = -x;

        yIn = y;
        xIn = x;
        rxIn = rx;
        initialStopPower = speedScalar;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        setLeftFrontPower(((-y + x + rx) / denominator) * speedScalar);
        setLeftBackPower(((y + x + rx) / denominator) * speedScalar);
        setRightFrontPower(((-y + x - rx) / denominator) * speedScalar);
        setRightBackPower(((y + x - rx) / denominator) * speedScalar);

        ABSTimer.reset();
    }

    public void BrakeABS(){
        if(ABSTimer.milliseconds() <= TOTAL_BRAKE_CYCLES * ((BRAKE_CYCLE + FLOAT_CYCLE))) {
            brakePowerScalar = initialStopPower - (brakeCycleNum / TOTAL_BRAKE_CYCLES) * initialStopPower; // drop power down according to cycle num

            if (ABSTimer.milliseconds() < brakeCycleNum * BRAKE_CYCLE) {
                Move(yIn, xIn, rxIn, brakePowerScalar);
            }

            if (ABSTimer.milliseconds() < brakeCycleNum * (BRAKE_CYCLE + FLOAT_CYCLE) && !floatMode) {
                floatMode = true;
                brakeMode = false;
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Stop();
            }

            if (ABSTimer.milliseconds() >= brakeCycleNum * (BRAKE_CYCLE + FLOAT_CYCLE)) {
                brakeCycleNum++;
            }

            if (brakeCycleNum >= TOTAL_BRAKE_CYCLES && !brakeMode) {
                brakeMode = true;
                floatMode = false;
                brakeCycleNum = 0;
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        Stop();
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
