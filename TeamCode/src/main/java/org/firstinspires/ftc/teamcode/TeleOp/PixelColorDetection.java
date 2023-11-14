package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.RoboMom;

@TeleOp(name = "PixelColorDetection", group = "Z")
public class PixelColorDetection extends RoboMom {
    NormalizedColorSensor backColorSensor;
    NormalizedColorSensor frontColorSensor;
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

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA backColors = backColorSensor.getNormalizedColors();
            Color.colorToHSV(backColors.toColor(), backHSVValues);

            NormalizedRGBA frontColors = backColorSensor.getNormalizedColors();
            Color.colorToHSV(backColors.toColor(), frontHSVValues);

            //Checks if there is a pixel in the front slot
            if (frontHSVValues[2]>0.01) {
                //Sets LEDs to the color of the pixel
                if (frontHSVValues[0] > 80 && frontHSVValues[0] < 110) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
                    telemetry.addLine("The pixel is yellow");
                } else if (frontHSVValues[0] > 110 && frontHSVValues[0] < 130) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    telemetry.addLine("The pixel is green");
                } else if (frontHSVValues[0] > 140 && frontHSVValues[0] < 165) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                    telemetry.addLine("The pixel is white");
                } else if (frontHSVValues[0] > 170 && frontHSVValues[0] < 200) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                    telemetry.addLine("The is purple");
                }

            //Checks if there is a pixel in the back slot
            } else if (backHSVValues[2]>0.01) {
                //Sets LEDs to the color of the pixel
                if (backHSVValues[0] > 80 && backHSVValues[0] < 110) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
                    telemetry.addLine("The pixel is yellow");
                } else if (backHSVValues[0] > 110 && backHSVValues[0] < 130) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    telemetry.addLine("The pixel is green");
                } else if (backHSVValues[0] > 140 && backHSVValues[0] < 165) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                    telemetry.addLine("The pixel is white");
                } else if (backHSVValues[0] > 170 && backHSVValues[0] < 200) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                    telemetry.addLine("The pixel is purple");
                }
            //Turns LEDs off if there is no pixel detected
            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                telemetry.addLine("There is no pixel");
            }

            blinkinLedDriver.setPattern(pattern);

            telemetry.addData("Pattern: ", pattern.toString());
            telemetry.addLine()
                    .addData("Red", "%.3f", backColors.red)
                    .addData("Green", "%.3f", backColors.green)
                    .addData("Blue", "%.3f", backColors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", backHSVValues[0])
                    .addData("Saturation", "%.3f", backHSVValues[1])
                    .addData("Value", "%.3f", backHSVValues[2]);
            telemetry.update();

        }


    }   // end method runOpMode()


}   // end class
