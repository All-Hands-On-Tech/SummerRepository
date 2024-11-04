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

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(15.2, -62, Math.toRadians(90)));
        delivery = new Delivery(this, false);
        intake = new Intake(this);

        delivery.clawClose();

        Action trajToScoreFirstSample;
        Action trajToCollectSamples;
        Action trajToCollectSecondSample;
        Action trajToScoreSecondSample1;
        Action trajToScoreSecondSample2;
        Action trajToPark;



        trajToScoreFirstSample = drive.actionBuilder(drive.pose)
                //Scores pre set specimin
                .strafeTo(new Vector2d(9, -33))
                /*score specimin*/
                .build();

        trajToCollectSamples = drive.actionBuilder(new Pose2d(9, -33, Math.toRadians(90)))
                //Brings two samples to observation zone
                .strafeTo(new Vector2d(25, -40))
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
                //.turnTo(Math.toRadians(-80))
                .build();

        trajToCollectSecondSample = drive.actionBuilder(new Pose2d(39, -49, Math.toRadians(-90)))
                //Scores a second specimin
                /*sleep*/
                .splineToConstantHeading(new Vector2d(40.5, -61.5), Math.toRadians(-90))
                .build();

        trajToScoreSecondSample1 = drive.actionBuilder(new Pose2d(40.5, -61.5, Math.toRadians(-90)))
                /*grab specimin*/
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(6, -50, Math.toRadians(95)), Math.toRadians(90))
                /*score specimin*/
                .build();

        trajToScoreSecondSample2 = drive.actionBuilder(new Pose2d(6, -50, Math.toRadians(95)))
                /*grab specimin*/
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(6, -33), Math.toRadians(-90))
                /*score specimin*/
                .build();

        trajToPark = drive.actionBuilder(new Pose2d(6, -34,Math.toRadians(90)))
                //Returns to observation zone
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
                        SlideToHeightAndIntakeToAngleAction(2100, VERTICAL_INTAKE_POS)
                )
        );
//        intake.updateAngle();
//        delivery.clawToTarget(1650, 5);
        intakeAndDeliveryToPosition(1650, 5, VERTICAL_INTAKE_POS);
        delivery.clawOpen();
        sleep(300);
//        delivery.clawToTarget(1850, 3);
        intakeAndDeliveryToPosition(1850, 3, VERTICAL_INTAKE_POS);


        //This code brings two samples to the Obersvation Zone
        Actions.runBlocking(
                new ParallelAction(
                        trajToCollectSamples,
                        SlideToHeightAndIntakeToAngleAction(30, VERTICAL_INTAKE_POS)

                )
        );


        //This code collects the second specimen
//        intake.updateAngle();
//        delivery.clawToTarget(800, 2);
        intakeAndDeliveryToPosition(800, 2, VERTICAL_INTAKE_POS);
        sleep(900);
        Actions.runBlocking(trajToCollectSecondSample);
        delivery.clawClose();
        sleep(400);

        //This code scores the second specimen
        Actions.runBlocking(
                new ParallelAction(
                        trajToScoreSecondSample1,
                        SlideToHeightAndIntakeToAngleAction(2200, VERTICAL_INTAKE_POS)
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        trajToScoreSecondSample2,
                        SlideToHeightAndIntakeToAngleAction(2200, VERTICAL_INTAKE_POS)
                )
        );
//        intake.updateAngle();
//        delivery.clawToTarget(1650, 3);
        intakeAndDeliveryToPosition(1650, 3, VERTICAL_INTAKE_POS);
        delivery.clawOpen();
        sleep(450);
        sleep(50);
        
        Actions.runBlocking(
                new ParallelAction(
                        trajToPark,
                        SlideToHeightAndIntakeToAngleAction(1500, VERTICAL_INTAKE_POS)
                )
        );
//        delivery.clawToTarget(0, 5);
        intakeAndDeliveryToPosition(0, 5, VERTICAL_INTAKE_POS);
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