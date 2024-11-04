package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Obs", group = "Autonomous")
public class Obs extends LinearOpMode {
    Delivery delivery = null;
    Intake intake = null;

    private static final int VERTICAL_INTAKE_POS = -230;
    private static final int CLAW_FLOOR = 0;
    private static final int CLAW_COLLECT = 800;
    private static final int CLAW_SCORE = 1650;
    private static final int CLAW_HIGH_RUNG = 2200;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(15.2, -62, Math.toRadians(90)));
        delivery = new Delivery(this, false);
        intake = new Intake(this);

        delivery.clawClose();

        Action trajToScoreFirstSample;
        Action trajToCollectSamples;
        Action trajToCollectAdditionalSample;
        Action trajToPrepareAdditionalSample;
        Action trajToScoreAdditionalSample;
        Action trajToReturnAfterAdditionalSample;
        Action trajToPark;



        trajToScoreFirstSample = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(9, -33), Math.toRadians(90))
                .build();

        trajToCollectSamples = drive.actionBuilder(new Pose2d(9, -33, Math.toRadians(90)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(25, -37, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(36, -15,Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(new Pose2d(44, -58, Math.toRadians(90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(42, -12, Math.toRadians(180)), Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(55, -58), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(39, -49), Math.toRadians(-90))
                .build();

        trajToCollectAdditionalSample = drive.actionBuilder(new Pose2d(39, -49, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(40.5, -61.5), Math.toRadians(-90))
                .build();
        trajToPrepareAdditionalSample = drive.actionBuilder(new Pose2d(40.5, -61.5, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(6, -50, Math.toRadians(95)), Math.toRadians(90))
                .build();
        trajToScoreAdditionalSample = drive.actionBuilder(new Pose2d(6, -50, Math.toRadians(95)))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(6, -33), Math.toRadians(-90))
                .build();
        trajToReturnAfterAdditionalSample = drive.actionBuilder(new Pose2d(6, -33, Math.toRadians(95)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(39, -49, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        trajToPark = drive.actionBuilder(new Pose2d(39, -49, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-90))
                .splineTo(new Vector2d(40, -57), Math.toRadians(-45))
                .build();

        intake.brakePitch();

        waitForStart();

        intake.setTargetLengthServo(0.2);

        //This code scores preloaded specimen
        Actions.runBlocking(
                new ParallelAction(
                        trajToScoreFirstSample,
                        SlideToHeightAndIntakeToAngleAction(CLAW_HIGH_RUNG, VERTICAL_INTAKE_POS)
                )
        );
//        intake.updateAngle();
//        delivery.clawToTarget(1650, 5);
        intakeAndDeliveryToPosition(CLAW_SCORE, 5, VERTICAL_INTAKE_POS);
        delivery.clawOpen();
        sleep(200);
//        delivery.clawToTarget(1850, 3);
        intakeAndDeliveryToPosition(CLAW_SCORE, 3, VERTICAL_INTAKE_POS);


        //This code brings two samples to the Obersvation Zone
        Actions.runBlocking(
                new ParallelAction(
                        trajToCollectSamples,
                        SlideToHeightAndIntakeToAngleAction(CLAW_FLOOR, VERTICAL_INTAKE_POS)

                )
        );

        for (int i = 0; i>2; i++) {
            //This code collects another specimen
//          intake.updateAngle();
//          delivery.clawToTarget(800, 2);
            intakeAndDeliveryToPosition(CLAW_COLLECT, 2, VERTICAL_INTAKE_POS);
            sleep(400);
            Actions.runBlocking(trajToCollectAdditionalSample);
            delivery.clawClose();
            sleep(200);

            //This code scores another specimen
            Actions.runBlocking(
                    new ParallelAction(
                            trajToPrepareAdditionalSample,
                            SlideToHeightAndIntakeToAngleAction(CLAW_HIGH_RUNG, VERTICAL_INTAKE_POS)
                    )
            );
            Actions.runBlocking(
                    new ParallelAction(
                            trajToScoreAdditionalSample,
                            SlideToHeightAndIntakeToAngleAction(CLAW_HIGH_RUNG, VERTICAL_INTAKE_POS)
                    )
            );
//          intake.updateAngle();
//          delivery.clawToTarget(1650, 3);
            intakeAndDeliveryToPosition(CLAW_SCORE, 3, VERTICAL_INTAKE_POS);
            delivery.clawOpen();
            sleep(200);
            Actions.runBlocking(
                    new ParallelAction(
                            trajToReturnAfterAdditionalSample,
                            SlideToHeightAndIntakeToAngleAction(CLAW_COLLECT, VERTICAL_INTAKE_POS)
                    )
            );
        }
        
        Actions.runBlocking(
                new ParallelAction(
                        trajToPark,
                        SlideToHeightAndIntakeToAngleAction(CLAW_FLOOR, VERTICAL_INTAKE_POS)
                )
        );
//        delivery.clawToTarget(0, 5);
        intakeAndDeliveryToPosition(CLAW_FLOOR, 5, VERTICAL_INTAKE_POS);
        intake.unbrakePitch();

        if (isStopRequested()) return;

    }



    private void intakeAndDeliveryToPosition(int heightTarget, double heightPower, int pitchTarget){
        delivery.setSlidesTargetPosition(heightTarget);
        intake.setTargetAngleTicks(pitchTarget);

        while(delivery.clawIsFarFromTarget()){
            delivery.PControlPower(heightPower);
            intake.updateAngle();
        }

        delivery.setSlidesPower(0);
        intake.brakePitch();
    }

    public class SlideToHeightAndIntakeToAngleRR implements Action {
        private boolean initialized = false;
        private int targetHeight;
        private int targetPitch;

        public SlideToHeightAndIntakeToAngleRR(int TargetHeight, int TargetPitch) {
            targetHeight = TargetHeight;
            targetPitch = TargetPitch;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                delivery.setSlidesTargetPosition(targetHeight);
                intake.setTargetAngleTicks(targetPitch);
                initialized = true;
            }

            double pos = delivery.getMotorPosition();
            packet.addLine("In RR action");
            packet.addLine("Claw height:");
            packet.put("liftPos", pos);
            if (Math.abs(pos - targetHeight) > 20) {
                delivery.PControlPower(3);
                intake.setTargetAngleTicks(targetPitch);
                intake.updateAngle();
                return true;
            } else {
                delivery.setSlidesPower(0);
                intake.brakePitch();
                return false;
            }
        }
    }
    public Action SlideToHeightAndIntakeToAngleAction(int heightInTicks, int angleInTicks) {
        return new Obs.SlideToHeightAndIntakeToAngleRR(heightInTicks, angleInTicks);
    }
}