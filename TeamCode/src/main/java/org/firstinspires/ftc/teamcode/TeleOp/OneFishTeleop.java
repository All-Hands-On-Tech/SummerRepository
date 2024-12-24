package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.OneFishIntake;
import org.firstinspires.ftc.teamcode.OneFishSampleDelivery;
import org.firstinspires.ftc.teamcode.OneFishSpecimenDelivery;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="OneFish Teleop", group="AAA")
public class OneFishTeleop extends LinearOpMode {

    private ElapsedTime deliveryTimer = new ElapsedTime();

    DrivetrainFunctions drivetrainFunctions = null;
    OneFishSampleDelivery sampleDelivery = null;
    OneFishSpecimenDelivery specimenDelivery = null;

    OneFishIntake intake = null;


    private final double HOME_X = 50;
    private final double HOME_Y = -20;

    private final double STORE_X = -35.94;
    private final double STORE_Y = 20.75;

    private double xOffset = 0.0;
    private double yOffset = 0.0;

    private Pose2d initPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
    private Pose2d prevPoseEstimate = new Pose2d(0,0,0);
    private Pose2d poseEstimate = new Pose2d(0,0,0);

    private boolean isGlobalTargeting = false;
    private double prevEncoderX = 0.0;
    private double encoderX = 0.0;
    private double deltaX = 0.0;

    private int targetExtention = 0;

    private boolean controlsRelinquished = false;
    private final double DRIVE_DEADZONE = 0.05;
    private final double SCORE_SPEED_SCALAR = 0.2;

    private double speedScalar = 1;
    private int sampleDeliveryHeight;
    private final int HEIGHT_INCREMENT = 1;
    private final double INTAKE_EXTENSION_TIME = 1;
    private final double DELIVER_PITCH_TIME = 0.25;
    private final double PITCH_TO_DELIVER_TIME = 1;
    private final double SHAKE_TIME = 1.5;
    private final double TRANSFER_TIME = 0.5;
    private final double DUMP_TIME = 0.25;
    private final double DELIVERY_EXTENSION_TIME = DELIVER_PITCH_TIME + SHAKE_TIME + 1;
    private final double SPECIMEN_SCORE_TIME = 0.75;
    boolean transfered = false;
    boolean dumped = false;
    boolean retracted = false;

    private enum RobotState{
        INTAKE_EXTEND,
        INTAKE,
        TRANSFER,
        DELIVERY_EXTEND,
        DELIVERY_DUMP,
        SPECIMEN,
        SPECIMEN_SCORE,
        IDLE
    }

    RobotState state = RobotState.IDLE;

    //This is the code to add rr actions
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime transferTimer = new ElapsedTime();
    private ElapsedTime dumpTimer = new ElapsedTime();
    @Override
    public void runOpMode() {

        specimenDelivery = new OneFishSpecimenDelivery(this);

        drivetrainFunctions = new DrivetrainFunctions(this);
        intake = new OneFishIntake(this);
        sampleDelivery = new OneFishSampleDelivery(this, true);

//        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        waitForStart();

        while (opModeIsActive()){
//            drive.updatePoseEstimate();
//            poseEstimate = drive.pose;
//            encoderX = drivetrainFunctions.getX();

            TelemetryPacket packet = new TelemetryPacket();

            //driver 1
            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                speedScalar = 0.25;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            } else {
                speedScalar = 1;
            }

            if (!controlsRelinquished) {
                float leftX = gamepad1.left_stick_x;
                float leftY = gamepad1.left_stick_y;
                float rightX = gamepad1.right_stick_x;
                float rightY = gamepad1.right_stick_y;
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

            //driver 2
            if(!controlsRelinquished) {
                float leftX = gamepad2.left_stick_x;
                float leftY = -gamepad2.left_stick_y;
                float rightX = gamepad2.right_stick_x;
                float rightY = -gamepad2.right_stick_y;

//                if(Math.abs(leftY) > DRIVE_DEADZONE){
//                     = IntakeState.INTAKE;
//                }
//
//                if(gamepad2.x){
//                    intake.setIntakePower(1);
//                }else if(gamepad2.b){
//                    intake.setIntakePower(-1);
//                }else{
//                    intake.setIntakePower(0);
//                }
//
//                if(gamepad2.y){
//                    intakeState = IntakeState.EXTEND;
//                    intake.setTargetLength(intake.MAX_EXTENSION);
//                }
//
//                if(gamepad2.a){
//                    intakeState = IntakeState.EXTEND;
//                    intake.setTargetLength(intake.MIN_EXTENSION);
//                }
//
////                if(gamepad2.dpad_up){
////                    intake.resetEncoder();
////                }
//
//                if(gamepad2.dpad_left){
//                    intake.pitchDown();
//                }
//                if(gamepad2.dpad_right){
//                    intake.pitchToTransfer();
//                }
//
//                if(gamepad2.dpad_up){
//                    sampleDelivery.pitchToDeliver();
//                }
//                if(gamepad2.dpad_down){
//                    sampleDelivery.pitchToTransfer();
//                }
//                if(gamepad2.right_trigger > 0.05){
//                    sampleDelivery.clawClose();//WIP Reversed
//                } else{
//                    sampleDelivery.clawOpen();//WIP Reversed
//                }
//
//                if(gamepad2.left_bumper){
//                    specimenDelivery.pitchToIntake();
//                }
//                if(gamepad2.right_bumper){
//                    specimenDelivery.pitchToDelivery();
//                }
//
//                if(gamepad2.right_trigger > 0.05){
//                    specimenDelivery.clawOpen();
//                } else{
//                    specimenDelivery.clawClose();
//                }
//
//
//                if(Math.abs(rightY) > DRIVE_DEADZONE){
//                        sampleDeliveryHeight += gamepad2.right_stick_y * 35;
//                }
//



                switch (state){
                    case INTAKE:
                        sampleDelivery.pitchToTransfer();
                        float intakePitch = (float) intake.getPitch();
                        intake.runPower();

                        //slides
                        if(Math.abs(leftY) > DRIVE_DEADZONE){
                            intake.setExtensionPower(leftY);
                        }else{
                            intake.setExtensionPower(0);
                        }

                        //Pitch
                        if(Math.abs(rightY) > DRIVE_DEADZONE){
                            intakePitch -= (float) (rightY*0.01);
                            intake.setPitch(intakePitch);
                        }
                        telemetry.addData("Pitch: ", intakePitch);

                        //Intake
                        if(gamepad2.right_trigger> DRIVE_DEADZONE){
                            intake.setIntakePower(-gamepad2.right_trigger);
                        }
                        if(gamepad2.left_trigger> DRIVE_DEADZONE){
                            intake.setIntakePower(gamepad2.left_trigger);
                        }
                        if(gamepad2.right_trigger < DRIVE_DEADZONE && gamepad2.left_trigger < DRIVE_DEADZONE){
                            intake.setIntakePower(0);
                        }

                        //TO INTAKE_EXTEND
                        if(gamepad2.a){
                            timer.reset();
                            state = RobotState.INTAKE_EXTEND;
                            intake.setTargetLength(intake.MIN_EXTENSION);
                        }
                        if(gamepad2.y){
                            timer.reset();
                            state = RobotState.INTAKE_EXTEND;
                            intake.setTargetLength(intake.MAX_EXTENSION);
                        }
                    break;

                    case INTAKE_EXTEND:
                        intake.pitchUp();
                        intake.runToPosition();
                        intake.updateLength();

                        //break condition
                        if(timer.seconds() > INTAKE_EXTENSION_TIME){
                            state = RobotState.IDLE;
//                            intake.setIntakePower(0);
                        }

                        break;
                    case TRANSFER:
                        sampleDelivery.setSlidesTargetPosition(0);
                        sampleDelivery.PControlPower(1);
                        if(timer.seconds()>DELIVER_PITCH_TIME){
                            if(!transfered){
                                intake.pitchToTransfer();
                            }
                            if(gamepad2.a){
                                sampleDelivery.clawOpen();
                                transferTimer.reset();
                                transfered = true;
                            }
                        }
                        telemetry.addData("transferred: ", transfered);
                        if(transferTimer.seconds() > TRANSFER_TIME && transfered){
                            intake.pitchDown();
                            //transfered = false;
                            sampleDeliveryHeight = -2000;
                            state = RobotState.DELIVERY_EXTEND;
                            timer.reset();
                        }

                        break;
                    case DELIVERY_EXTEND:
                        transfered = false;
                        sampleDelivery.setSlidesTargetPosition(sampleDeliveryHeight);
                        sampleDelivery.PControlPower(1);
                        //pitch out to 90 to reorient sample
                        if(timer.seconds() > PITCH_TO_DELIVER_TIME && timer.seconds() < PITCH_TO_DELIVER_TIME + SHAKE_TIME){
                            sampleDelivery.pitchToShake();
                        }
                        //pitch back up to deliver to high basket
                        if(timer.seconds() > PITCH_TO_DELIVER_TIME + SHAKE_TIME){
                            sampleDelivery.pitchToDeliver();
                        }
                        //Get ready to dump
                        if(timer.seconds() > DELIVERY_EXTENSION_TIME){
                            state = RobotState.DELIVERY_DUMP;
                        }

                        break;

                    case DELIVERY_DUMP:

                        sampleDelivery.setSlidesTargetPosition(sampleDeliveryHeight);
                        sampleDelivery.PControlPower(1);

                        //Pitch back
                        if(dumped && dumpTimer.seconds() > DUMP_TIME){
                            sampleDeliveryHeight = 0;
                            sampleDelivery.pitchToTransfer();
                            timer.reset();
                            retracted = true;
                        }

                        //wait for right trigger to score
                        if(gamepad2.right_trigger > DRIVE_DEADZONE && !dumped){
                            sampleDelivery.clawClose();
                            dumpTimer.reset();
                            dumped = true;
                        }

                        //Switch to idle when retracted
                        if(timer.seconds() > DELIVERY_EXTENSION_TIME && retracted){
                            state = RobotState.IDLE;
                            retracted = false;
                            dumped = false;
                        }

                        telemetry.addData("Retracted: ", retracted);
                        telemetry.addData("retract time: ", timer.seconds());

                        break;
                    case SPECIMEN:
                        intake.pitchUp();
//                        sampleDelivery.pitchToAway();
                        if(gamepad2.a) {
                            timer.reset();
                            specimenDelivery.pitchToIntake();
                            state = RobotState.SPECIMEN_SCORE;
                        }
                        if(gamepad2.b) {
                            specimenDelivery.pitchToDelivery();
                        }
                        if(gamepad2.right_bumper){
                            specimenDelivery.pitchToVertical();
                        }
                        if(gamepad2.y) {
                            specimenDelivery.clawClose();
                        }
                        if(gamepad2.x) {
                            specimenDelivery.clawOpen();
                        }
                        if(gamepad2.dpad_down) {
                            state = RobotState.IDLE;
                        }
                        break;

                    case SPECIMEN_SCORE:
                        if(timer.seconds() > SPECIMEN_SCORE_TIME){
                            specimenDelivery.clawOpen();
                            intake.pitchUp();
                            sampleDelivery.pitchToTransfer();
                            state = RobotState.IDLE;
                        }
                        break;

                    case IDLE:
                        //TO INTAKE_EXTEND
                        if(gamepad2.a){
                            timer.reset();
                            state = RobotState.INTAKE_EXTEND;
                            intake.setTargetLength(intake.MIN_EXTENSION);
                        }
                        if(gamepad2.y){
                            timer.reset();
                            state = RobotState.INTAKE_EXTEND;
                            intake.setTargetLength(intake.MAX_EXTENSION);
                        }
                        if(gamepad2.right_trigger > DRIVE_DEADZONE){
                            state = RobotState.INTAKE;
                        }
                        //TO TRANSFER
                        if(gamepad2.x){
                            timer.reset();
                            state = RobotState.TRANSFER;
                            sampleDelivery.pitchToTransfer();
                            sampleDelivery.clawClose();
                        }
                        //TO SPECIMEN
                        if(gamepad2.dpad_up) {
                            state = RobotState.SPECIMEN;
                        }
                        break;
                }

                if(gamepad2.back){
                    state = RobotState.IDLE;
                }

                telemetry.addData("STATE: ", state);
                telemetry.addData("TRANSFERTIMER", transferTimer.seconds());


            }

            telemetry.update();

            //Adds rr actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

//            prevPoseEstimate = poseEstimate;
            prevEncoderX = encoderX;
        }
    }
}
