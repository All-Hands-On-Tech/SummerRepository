package org.firstinspires.ftc.teamcode.George;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlyWheels {

    LinearOpMode linearOpMode;
    DcMotor left, right;
    int initAttempts;
    boolean isDisabled = false;
    public FlyWheels(LinearOpMode l)
    {
        initAttempts = 0;

        linearOpMode = l;
        Initialize();

    }


    private void Initialize(){
        try {
            left = linearOpMode.hardwareMap.get(DcMotor.class, "leftFly");
            right = linearOpMode.hardwareMap.get(DcMotor.class, "rightFly");


            left.setDirection(DcMotor.Direction.REVERSE);
            right.setDirection(DcMotor.Direction.FORWARD);

            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find motors.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }
    public void Reinitialize(){
        try {
            left = linearOpMode.hardwareMap.get(DcMotor.class, "leftFly");
            right = linearOpMode.hardwareMap.get(DcMotor.class, "rightFly");


            left.setDirection(DcMotor.Direction.REVERSE);
            right.setDirection(DcMotor.Direction.FORWARD);


        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find motors.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void Power(float power){
        left.setPower(power);
        right.setPower(power);
    }

    public void Pulse(float power, float pulseLen, ElapsedTime timer){
        timer.reset();
        while(timer.seconds() < pulseLen){
            left.setPower(power);
            right.setPower(power);
        }
        left.setPower(0);
        right.setPower(0);
    }

}
