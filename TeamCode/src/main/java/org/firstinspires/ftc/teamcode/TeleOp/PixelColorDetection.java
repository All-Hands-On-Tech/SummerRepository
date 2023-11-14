package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import java.util.List;

@TeleOp(name = "PixelColorDetection", group = "Z")
public class PixelColorDetection extends RoboMom {
    NormalizedColorSensor colorSensor;
    final float[] hsvValues = new float[3];

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(10);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            //all of the always false if statements will be replaced by the hsv for the pixels
            if (hsvValues[2]<0.01) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                telemetry.addLine("There is no pixel");
            } else if (hsvValues[0] > 80 && hsvValues[0] < 110) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
                telemetry.addLine("The pixel is yellow");
            } else if (hsvValues[0] > 110 && hsvValues[0] < 130) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                telemetry.addLine("The pixel is green");
            } else if (hsvValues[0] > 140 && hsvValues[0] < 165) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                telemetry.addLine("The pixel is white");
            } else if (hsvValues[0] > 170 && hsvValues[0] < 200) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                telemetry.addLine("The pixel is purple");
            }

            blinkinLedDriver.setPattern(pattern);

            telemetry.addData("Pattern: ", pattern.toString());
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.update();

            telemetry.addData("Connection info", blinkinLedDriver.getConnectionInfo());

        }


    }   // end method runOpMode()


}   // end class
