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
    final float[] backHSVValues = new float[3];
    final float[] frontHSVValues = new float[3];

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

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

            if (((DistanceSensor) frontColorSensor).getDistance(DistanceUnit.CM) < 3) {
                //Checks if there is a pixel in the front slot
                if (frontHSVValues[0] > 80 && frontHSVValues[0] < 110) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
                    frontColor = "YELLOW";
                } else if (frontHSVValues[0] > 110 && frontHSVValues[0] < 140) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    frontColor = "GREEN";
                } else if (frontHSVValues[0] > 150 && frontHSVValues[0] < 180) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                    frontColor = "WHITE";
                } else if (frontHSVValues[0] > 200 && frontHSVValues[0] < 220) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                    frontColor = "PURPLE";
                } else {
                    frontColor = "no pixel";
                }
            } else if (((DistanceSensor) backColorSensor).getDistance(DistanceUnit.CM) < 3) {
                if (backHSVValues[0] > 80 && backHSVValues[0] < 110) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
                    backColor = "YELLOW";
                } else if (backHSVValues[0] > 110 && backHSVValues[0] < 140) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    backColor = "GREEN";
                } else if (backHSVValues[0] > 150 && backHSVValues[0] < 180) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                    backColor = "WHITE";
                } else if (backHSVValues[0] > 200 && backHSVValues[0] < 220) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                    backColor = "PURPLE";
                } else {
                    backColor = "no pixel";
                }
            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }

            blinkinLedDriver.setPattern(pattern);

            telemetry.addData("Pattern: ", pattern.toString());
            telemetry.addLine("Back sensor");
            telemetry.addLine()
                    .addData("Red", "%.3f", backColors.red)
                    .addData("Green", "%.3f", backColors.green)
                    .addData("Blue", "%.3f", backColors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", backHSVValues[0])
                    .addData("Saturation", "%.3f", backHSVValues[1])
                    .addData("Value", "%.3f", backHSVValues[2]);
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) backColorSensor).getDistance(DistanceUnit.CM));
            telemetry.addLine("Front sensor");
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
