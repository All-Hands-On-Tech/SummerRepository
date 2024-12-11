package org.firstinspires.ftc.teamcode.TeleOp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.util.*;

@TeleOp(name="Husky Lens Experiment", group="A")
public class testingHuskyLens extends LinearOpMode {
    DrivetrainFunctions drivetrainFunctions = null;
    Intake intake = null;

    private final double DRIVE_DEADZONE = 0.05;
    private double speedScalar = 0.6;

    private int targetPitch = 0;
    private double extension = 0.25;
    private int PITCH_INCREMENT = 15;
    private int lowHeight = -300;

    // Declare OpMode members.
    private HuskyLens husky;

    final int imageWidth = 320;
    final int imageHeight = 240;
    final int centerX = 160;
    final int centerY = 180;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() {
        drivetrainFunctions = new DrivetrainFunctions(this);
        intake = new Intake(this);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));

        husky = hardwareMap.get(HuskyLens.class, "husky");
        husky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.pose;
            TelemetryPacket packet = new TelemetryPacket();
            boolean toSample = gamepad1.a;

            if (!(toSample || !runningActions.isEmpty())) {
                float leftX = -gamepad1.left_stick_x;
                float leftY = -gamepad1.left_stick_y;
                float rightX = -gamepad1.right_stick_x;
                float rightY = -gamepad1.right_stick_y;
                if (Math.abs(leftX) > DRIVE_DEADZONE || Math.abs(leftY) > DRIVE_DEADZONE || Math.abs(rightX) > DRIVE_DEADZONE || Math.abs(rightY) > DRIVE_DEADZONE*2) {
                    if(Math.abs(rightY) > DRIVE_DEADZONE*2) {
                        drivetrainFunctions.Move(leftX, rightY, rightX, speedScalar);
                    }else{
                        drivetrainFunctions.Move(leftX, leftY, rightX, speedScalar);
                    }
                } else {
                    drivetrainFunctions.Stop();
                }
            }

            float rightTrigger = gamepad1.right_trigger;
            float leftTrigger = gamepad1.left_trigger;
            if(rightTrigger > 0){
                extension += rightTrigger/100;
                extension = Math.max(-0.1, Math.min(0.25, extension));
                intake.setTargetLengthServo(extension);
            } else if(leftTrigger > 0){
                extension -= leftTrigger/100;
                extension = Math.max(-0.1, Math.min(0.25, extension));
                intake.setTargetLengthServo(extension);
            }

            intake.updateAngle();
            intake.updateLength();

            if(gamepad1.left_bumper){
                intake.setEndEffectorPosition(0.375f);//in
            }else if (gamepad1.right_bumper){
                intake.setEndEffectorPosition(0.75f);//out
            }


            if(gamepad1.dpad_up){
                intake.incrementTargetAngleTicks(-PITCH_INCREMENT);
            } else if(gamepad1.dpad_down){
                intake.incrementTargetAngleTicks(+PITCH_INCREMENT);
            } else if(gamepad1.dpad_left) {
                intake.setTargetAngleTicks(lowHeight+50);
                extension = 0.01;
                intake.setTargetLengthServo(extension);
                intake.setEndEffectorPosition(0.75f);
            } else if(gamepad1.dpad_right) {
                lowHeight = intake.getTargetAngleTicks();
            }


            HuskyLens.Block[] huskyBlocks = husky.blocks();

            ArrayList<HuskyLens.Block> yellowBlocks = new ArrayList<HuskyLens.Block>();
            ArrayList<HuskyLens.Block> redBlocks = new ArrayList<HuskyLens.Block>();
            ArrayList<HuskyLens.Block> blueBlocks = new ArrayList<HuskyLens.Block>();

            //telemetry.addData("Block count", huskyBlocks.length);
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

                deltaX = deltaX/imageWidth;
                deltaY = deltaY/imageHeight;
                if (deltaX > 0) {
                    telemetry.addData("move right", deltaX);
                } else {
                    telemetry.addData("move left", deltaX);
                }
                if (deltaY > 0) {
                    telemetry.addData("move back", deltaY);
                } else {
                    telemetry.addData("move forward", deltaY);
                }

                if (toSample) {
                    drivetrainFunctions.Move((float) -deltaX, (float) -deltaY, 0, speedScalar);
                }


                telemetry.addLine("\n");


//                for (HuskyLens.Block block : yellowBlocks) {
//                    telemetry.addLine(block.toString());
//                }
            }

            if (gamepad1.b) {
                runningActions.add(new SequentialAction(
                        //new InstantAction(() ->  drivetrainFunctions.Move(0, 1, 0, speedScalar/3)),
                        //new SleepAction(0.1),
                        //new InstantAction(() -> drivetrainFunctions.Stop()),
                        new InstantAction(() -> extension = 0.25),
                        new InstantAction(() -> intake.setTargetLengthServo(extension)),
                        new SleepAction(1),
                        new InstantAction(() -> intake.setTargetAngleTicks(lowHeight+30)),
                        new SleepAction(1),
                        new InstantAction(() -> intake.setEndEffectorPosition(0.375f))

                ));
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                telemetry.addData("Action: ", action.toString());
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);



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
