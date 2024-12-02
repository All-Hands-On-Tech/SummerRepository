package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.*;

@TeleOp(name="Husky Lens Experiment", group="A")
public class testingHuskyLens extends LinearOpMode {

    // Declare OpMode members.
    private HuskyLens husky;

    final int imageWidth = 320;
    final int imageHeight = 240;
    final int centerX = 160;
    final int centerY = 120;

    @Override
    public void runOpMode() {
        husky = hardwareMap.get(HuskyLens.class, "husky");

        husky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] huskyBlocks = husky.blocks();

            ArrayList<HuskyLens.Block> yellowBlocks = new ArrayList<HuskyLens.Block>();
            ArrayList<HuskyLens.Block> redBlocks = new ArrayList<HuskyLens.Block>();
            ArrayList<HuskyLens.Block> blueBlocks = new ArrayList<HuskyLens.Block>();

            telemetry.addData("Block count", huskyBlocks.length);
            for (HuskyLens.Block sample : huskyBlocks) {
                if (sample.id == 1) {
                    yellowBlocks.add(sample);
                } else if (sample.id == 2) {
                    redBlocks.add(sample);
                } else if (sample.id == 3) {
                    blueBlocks.add(sample);
                }
            }

            if (!yellowBlocks.isEmpty()) {
                Collections.sort(yellowBlocks, new Comparator<HuskyLens.Block>() {
                    public int compare(HuskyLens.Block a, HuskyLens.Block b) {
                        return distanceToCenter(a) - distanceToCenter(b);
                    }
                });

                double deltaX = yellowBlocks.get(0).x - centerX;
                double deltaY = yellowBlocks.get(0).y - centerY;
                if (deltaX > 0) {
                    telemetry.addData("move right", deltaX/imageWidth);
                } else {
                    telemetry.addData("move left", deltaX/imageWidth);
                }
                if (deltaY > 0) {
                    telemetry.addData("move back", deltaY/imageHeight);
                } else {
                    telemetry.addData("move forward", deltaY/imageHeight);
                }


                telemetry.addLine("\n");


                for (HuskyLens.Block block : yellowBlocks) {
                    telemetry.addLine(block.toString());
                }
            }
            telemetry.update();

        }
    }


    int distanceToCenter (HuskyLens.Block block) {
        return (int) Math.sqrt(Math.pow(block.x - centerX,2) + Math.pow(block.y - centerY,2));
    }

    class sortBlocks implements Comparator<HuskyLens.Block> {
        // Used for sorting in ascending order of
        // roll number
        public int compare(HuskyLens.Block a, HuskyLens.Block b){
            double distanceA = Math.sqrt(Math.pow(a.x,2) + Math.pow(a.y,2));
            double distanceB = Math.sqrt(Math.pow(b.x,2) + Math.pow(b.y,2));
            return (int) (distanceA - distanceA);
        }
    }
}
