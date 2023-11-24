package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoboMom;

@TeleOp(name = "PixelColorDetection", group = "Z")
public class PixelColorDetection extends RoboMom {
    NormalizedColorSensor backColorSensor;
    String backColor = "no pixel";
    NormalizedColorSensor frontColorSensor;
    String frontColor = "no pixel";

    boolean useFrontSensor = false;
    final float[] backHSVValues = new float[3];
    final float[] frontHSVValues = new float[3];

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    double yellowBack = 80;
    double yellowFront = 110;
    double greenBack = 110;
    double greenFront = 140;
    double whiteBack = 150;
    double whiteFront = 180;
    double purpleBack = 200;
    double purpleFront = 220;

    @Override
    public void runOpMode() {
        backColorSensor = hardwareMap.get(NormalizedColorSensor.class, "back_color");
        backColorSensor.setGain(10);
        frontColorSensor = hardwareMap.get(NormalizedColorSensor.class, "front_color");
        frontColorSensor.setGain(10);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA backColors = backColorSensor.getNormalizedColors();
            Color.colorToHSV(backColors.toColor(), backHSVValues);

            NormalizedRGBA frontColors = frontColorSensor.getNormalizedColors();
            Color.colorToHSV(frontColors.toColor(), frontHSVValues);

            if (((DistanceSensor) backColorSensor).getDistance(DistanceUnit.CM) < 3) {
                useFrontSensor = false;
                if (backHSVValues[0] > yellowBack && backHSVValues[0] < yellowFront) {
                    backColor = "YELLOW";
                } else if (backHSVValues[0] > greenBack && backHSVValues[0] < greenFront) {
                    backColor = "GREEN";
                } else if (backHSVValues[0] > whiteBack && backHSVValues[0] < whiteFront) {
                    backColor = "WHITE";
                } else if (backHSVValues[0] > purpleBack && backHSVValues[0] < purpleFront) {
                    backColor = "PURPLE";
                } else {
                    backColor = "color not detected";
                }
            } else {
                backColor = "no pixel";
            }

            if (((DistanceSensor) frontColorSensor).getDistance(DistanceUnit.CM) < 3) {
                useFrontSensor = true;
                if (frontHSVValues[0] > yellowBack && frontHSVValues[0] < yellowFront) {
                    frontColor = "YELLOW";
                } else if (frontHSVValues[0] > greenBack && frontHSVValues[0] < greenFront) {
                    frontColor = "GREEN";
                } else if (frontHSVValues[0] > whiteBack && frontHSVValues[0] < whiteFront) {
                    frontColor = "WHITE";
                } else if (frontHSVValues[0] > purpleBack && frontHSVValues[0] < purpleFront) {
                    frontColor = "PURPLE";
                } else {
                    backColor = "color not detected";
                }
            } else {
                backColor = "no pixel";
            }


            if (useFrontSensor) {setLEDColor(frontColor);}
            else {setLEDColor(backColor);}

            telemetry.addData("Pattern: ", pattern.toString());
            telemetry.addLine("Back sensor");
            telemetry.addLine(backColor);
            telemetry.addLine()
                    .addData("Red", "%.3f", backColors.red)
                    .addData("Green", "%.3f", backColors.green)
                    .addData("Blue", "%.3f", backColors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", backHSVValues[0])
                    .addData("Saturation", "%.3f", backHSVValues[1])
                    .addData("Value", "%.3f", backHSVValues[2]);
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) backColorSensor).getDistance(DistanceUnit.CM));
            telemetry.addLine("---------------");
            telemetry.addLine("Front sensor");
            telemetry.addLine(frontColor);
            telemetry.addLine()
                    .addData("Red", "%.3f", frontColors.red)
                    .addData("Green", "%.3f", frontColors.green)
                    .addData("Blue", "%.3f", frontColors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", frontHSVValues[0])
                    .addData("Saturation", "%.3f", frontHSVValues[1])
                    .addData("Value", "%.3f", frontHSVValues[2]);
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) frontColorSensor).getDistance(DistanceUnit.CM));
            telemetry.update();

        }


    }   // end method runOpMode()


}   // end class
