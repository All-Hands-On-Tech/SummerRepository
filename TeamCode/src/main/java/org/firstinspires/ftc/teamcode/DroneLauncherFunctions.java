package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Disabled
public class DroneLauncherFunctions {
    public Servo droneServo;

    private LinearOpMode linearOpMode;

    public boolean isDisabled = false;

    private double initAttempts = 0;

    private static final double RELEASE_POSITION = 0.6;

    private static final double DEFAULT_POSITION = 0.1;


    public DroneLauncherFunctions(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){
        try {
            droneServo = linearOpMode.hardwareMap.get(Servo.class, "droneServo");

        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find drone servo.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void Reinitialize(){
        try {
            droneServo = linearOpMode.hardwareMap.get(Servo.class, "droneServo");
            isDisabled = false;
        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find drone servo.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void ReleaseDrone(){
        droneServo.setPosition(RELEASE_POSITION);
    }

    public void initDroneServo(){droneServo.setPosition(DEFAULT_POSITION);}

}
