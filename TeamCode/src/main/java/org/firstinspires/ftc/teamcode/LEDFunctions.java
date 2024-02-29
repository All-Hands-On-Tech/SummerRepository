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
public class LEDFunctions {
    RevBlinkinLedDriver blinkinLedDriver;

    private LinearOpMode linearOpMode;

    public boolean isDisabled = false;

    private double initAttempts = 0;

    public static SampleMecanumDrive drive;

    public LEDFunctions(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){
        try {
            blinkinLedDriver = linearOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find motors.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void Reinitialize(){
        try {
            blinkinLedDriver = linearOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
            isDisabled = false;
        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find motors.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void setLEDColor (String colorFront, String colorBack, String extras) {
        if (extras == "BACK") {
            if (colorBack == "WHITE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);}
            if (colorBack == "YELLOW") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);}
            if (colorBack == "GREEN") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);}
            if (colorBack == "PURPLE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);}
            if (colorBack == "NONE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);}
        } else if (extras == "PARTY") {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        } else if (colorBack == "NONE") {
            if (colorFront == "WHITE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);}
            if (colorFront == "YELLOW") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);}
            if (colorFront == "GREEN") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);}
            if (colorFront == "PURPLE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);}
            if (colorFront == "NONE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);}
        } else if (colorFront == "NONE") {
            if (colorBack == "WHITE") {blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);}
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
