package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
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


@TeleOp(name="OneFishIndividualMethodTesting", group="TEST")
public class OneFishIndividualMethodTesting extends LinearOpMode {

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
    private final double TRANSFER_TIME = 0.5;
    private final double DUMP_TIME = 0.25;
    private final double DELIVERY_EXTENSION_TIME = 2;
    private final double PITCH_TO_DELIVER_TIME = 1;
    private final double SHAKE_TIME = 1.5;
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
            if(gamepad2.x){
                sampleDelivery.pitchToTransfer();
            }
            if(gamepad2.a){
                sampleDelivery.pitchToDeliver();
            }

            if(gamepad2.right_bumper){
                sampleDelivery.clawOpen();
            }
            if(gamepad2.left_bumper){
                sampleDelivery.clawClose();
            }
//            sampleDelivery.setClawPosition(gamepad2.right_trigger);
        }
    }
}
