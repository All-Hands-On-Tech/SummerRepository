package org.firstinspires.ftc.teamcode.teleOp;

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
    ArrayList<HuskyLens.Block> yellowBlocks = new ArrayList<HuskyLens.Block>();
    ArrayList<HuskyLens.Block> redBlocks = new ArrayList<HuskyLens.Block>();
    ArrayList<HuskyLens.Block> blueBlocks = new ArrayList<HuskyLens.Block>();

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
            telemetry.addData("Block count", huskyBlocks.length);
            for (HuskyLens.Block sample : huskyBlocks) {
                if (aspectRatio(sample) > 0.9) {
                    if (sample.id == 0) {
                        yellowBlocks.add(sample);
                    } else if (sample.id == 1) {
                        redBlocks.add(sample);
                    } else if (sample.id == 2) {
                        blueBlocks.add(sample);
                    }
                }
            }

            Collections.sort(yellowBlocks, new Comparator<HuskyLens.Block>(){
                public int compare(HuskyLens.Block a, HuskyLens.Block b){
                    return distanceToCenter(a) - distanceToCenter(b);
                }
            });

            if (yellowBlocks.get(0).x > centerX) {
                telemetry.addLine("move back");
            } else {
                telemetry.addLine("move forward");
            }

            if (yellowBlocks.get(0).y > centerY) {
                telemetry.addLine("move back");
            } else {
                telemetry.addLine("move forward");
            }

            telemetry.addLine("\n");
            

            for (HuskyLens.Block block : yellowBlocks) {
                telemetry.addLine(block.toString());
            }
            telemetry.update();

        }
    }

    double aspectRatio (HuskyLens.Block block) {
        return block.width/block.height;
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
