package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.OneFishIntake;
import org.firstinspires.ftc.teamcode.OneFishSampleDelivery;
import org.firstinspires.ftc.teamcode.OneFishSpecimenDelivery;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Net Onefish", group = "Autonomous")
public class Net extends LinearOpMode {
    OneFishSpecimenDelivery specimenDelivery = null;
    OneFishIntake intake = null;
    OneFishSampleDelivery sampleDelivery = null;

    DrivetrainFunctions driveTrain = null;

    private static final int VERTICAL_INTAKE_POS = -330;
    private static final int CLAW_FLOOR = 0;
    private static final int CLAW_COLLECT = 800;
    private static final int CLAW_SCORE = 1650;
    private static final int CLAW_HIGH_RUNG = 2200;

    private boolean imuInitialized = false;

    @Override
    public void runOpMode() {
        driveTrain = new DrivetrainFunctions(this, true);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-35.74, -62.12, Math.toRadians(90)));


        specimenDelivery = new OneFishSpecimenDelivery(this);
        intake = new OneFishIntake(this);
        sampleDelivery = new OneFishSampleDelivery(this, true);

        Action trajToNet;
        Action turnToIntakeSampleOne;
        Action turnToDeliverSampleOne;
        Action turnToIntakeSampleTwo;
        Action turnToDeliverSampleTwo;
        Action turnToIntakeSampleThree;
        Action turnToDeliverSampleThree;
        Action trajToPark;


        trajToNet = drive.actionBuilder(new Pose2d(-35.74, -62.12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-61, -54))
                .build();


        trajToPark = drive.actionBuilder(new Pose2d(-62, -55, Math.toRadians(0)))
                .strafeTo(new Vector2d(-40, -55))
                .strafeTo(new Vector2d(-40, -10))
                .strafeTo(new Vector2d(-24, -10))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(trajToNet, TransferSampleAction(-750)));
        Actions.runBlocking(DeliverSampleAction());
        Actions.runBlocking(new ParallelAction());

        //sampleDelivery.setSlidesTargetPosition(0);
        //                        sampleDelivery.PControlPower(1);
        //                        if(timer.seconds()>DELIVER_PITCH_TIME){
        //                            if(!transfered){
        //                                intake.pitchToTransfer();
        //                                intake.setIntakePower(-0.25f);
        //                            }
        //                            if(gamepad2.a){
        //                                sampleDelivery.clawOpen();
        //                                transferTimer.reset();
        //                                intake.setIntakePower(0);
        //                                transfered = true;
        //                            }
        //                        }
        //                        telemetry.addData("transferred: ", transfered);
        //                        if(transferTimer.seconds() > TRANSFER_TIME && transfered){
        //                            intake.pitchDown();
        //                            //transfered = false;
        //                            sampleDeliveryHeight = -2090;
        //                            state = RobotState.DELIVERY_EXTEND;
        //                            timer.reset();
        //                        }

//        Actions.runBlocking(trajToPark);


    }
    public class TransferSampleRR implements Action {
        private boolean initialized = false;
        private int targetHeight;
        private ElapsedTime timer = new ElapsedTime();
        private final double PREP_TIME = 250*2;
        private final double GRAB_TIME = 250*2;
        private final double CLEARANCE_TIME = 250*2;

        public TransferSampleRR(int TargetHeight) {
            targetHeight = TargetHeight;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                sampleDelivery.setSlidesTargetPosition(targetHeight);
                initialized = true;

            }

            if (timer.milliseconds() < PREP_TIME) {
                intake.pitchToTransfer();
                intake.setIntakePower(-0.25f);
                sampleDelivery.clawClose();
                return true;
            } else if (timer.milliseconds() > PREP_TIME && timer.milliseconds() < PREP_TIME + GRAB_TIME) {
                sampleDelivery.clawOpen();
                return true;
            } else if ((timer.milliseconds() > PREP_TIME + GRAB_TIME && timer.milliseconds() < PREP_TIME + GRAB_TIME + CLEARANCE_TIME)) {
                sampleDelivery.PControlPower(3);
                return true;
            } else {
                sampleDelivery.pitchToVertical();
                return false;
            }
        }
    }
    public Action TransferSampleAction(int heightInTicks) {
        return new Net.TransferSampleRR(heightInTicks);
    }

    public class DeliverSampleRR implements Action {
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();
        private final double PREP_TIME = 250*2;
        private final double DROP_TIME = 250*2;
        private final double CLEARANCE_TIME = 250*2;

        public DeliverSampleRR() {

        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                timer.reset();
            }

            if (timer.milliseconds() < PREP_TIME) {
                intake.pitchToTransfer();
                sampleDelivery.clawOpen();
                sampleDelivery.pitchToSpecimenDeliver();
                return true;
            } else if (timer.milliseconds() > PREP_TIME && timer.milliseconds() < PREP_TIME + DROP_TIME) {
                sampleDelivery.clawClose();
                return true;
            } else if ((timer.milliseconds() > PREP_TIME + DROP_TIME && timer.milliseconds() < PREP_TIME + DROP_TIME + CLEARANCE_TIME)) {
                sampleDelivery.pitchToVertical();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action DeliverSampleAction() {
        return new Net.DeliverSampleRR();
    }

    public class ResetSampleRR implements Action {
        private boolean initialized = false;
        private int targetHeight;
        private ElapsedTime timer = new ElapsedTime();
        private final double PREP_TIME = 250*2;
        private final double LOWER_TIME = 500*2;

        public ResetSampleRR() {
            targetHeight = 0;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                sampleDelivery.setSlidesTargetPosition(targetHeight);
                initialized = true;
                timer.reset();
            }

            if (timer.milliseconds() < PREP_TIME) {
                intake.pitchToTransfer();
                sampleDelivery.clawClose();
                sampleDelivery.pitchToTransfer();
                return true;
            } else if (timer.milliseconds() > PREP_TIME && timer.milliseconds() < PREP_TIME + LOWER_TIME) {
                sampleDelivery.PControlPower(3);
                return true;
            } else {
                return false;
            }
        }
    }
    public Action ResetSampleAction() {
        return new Net.ResetSampleRR();
    }
}