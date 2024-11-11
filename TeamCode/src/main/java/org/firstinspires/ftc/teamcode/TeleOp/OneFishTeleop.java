package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="OneFish Teleop", group="AAA")
public class OneFishTeleop extends LinearOpMode {

    private ElapsedTime deliveryTimer = new ElapsedTime();

    DrivetrainFunctions drivetrainFunctions = null;

    OneFishIntake intake = null;

    private final double CLICKS_PER_CM = 24.92788;

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

    private enum IntakeState{
        EXTEND,
        INTAKE
    }

    IntakeState intakeState = IntakeState.INTAKE;

    //This is the code to add rr actions
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    @Override
    public void runOpMode() {

        drivetrainFunctions = new DrivetrainFunctions(this);
        intake = new OneFishIntake(this);

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
                speedScalar = 0.5;
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

                if(Math.abs(leftY) > DRIVE_DEADZONE){
                    intakeState = IntakeState.INTAKE;
                }

                if(gamepad2.x){
                    intake.setIntakePower(1);
                }else if(gamepad2.b){
                    intake.setIntakePower(-1);
                }else{
                    intake.setIntakePower(0);
                }

                if(gamepad2.y){
                    intakeState = IntakeState.EXTEND;
                    intake.setTargetLength(intake.MAX_EXTENSION);
                }

                if(gamepad2.a){
                    intakeState = IntakeState.EXTEND;
                    intake.setTargetLength(intake.MIN_EXTENSION);
                }

                switch (intakeState){
                    case INTAKE:
                        intake.runPower();
                        if(Math.abs(leftY) > DRIVE_DEADZONE){
                            intake.setExtensionPower(leftY);
                        }else{
                            intake.setExtensionPower(0);
                        }
                    break;

                    case EXTEND:
                        intake.runToPosition();
                        intake.updateLength();
                }


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
