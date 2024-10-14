package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Obs", group = "Autonomous")
public class Obs extends LinearOpMode {
    Delivery delivery = null;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(15.2, -62, Math.toRadians(90)));
        delivery = new Delivery(this, false);

        Action trajToSubmersible;
        Action trajToCollectSamples;

        //FIXME:    SPLIT trajectoryAction1 INTO MULTIPLE ACTIONS SO DELIVERY ACTIONS CAN BE IMPLEMENTED IN PARALLEL / SEQUENTIALLY

        trajToSubmersible = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(7, -34))
                .build();

        trajToCollectSamples = drive.actionBuilder(new Pose2d(10, -34, Math.toRadians(90)))
                .strafeTo(new Vector2d(25, -45))
                .setTangent(-45)
                .splineToLinearHeading(new Pose2d(45, -14, Math.toRadians(90)), Math.toRadians(-70))
                .strafeTo(new Vector2d(45, -59))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(55.00, -11, Math.toRadians(90)), Math.toRadians(-90))
                .strafeTo(new Vector2d(55, -60))
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(62, -24.10, Math.toRadians(180)), Math.toRadians(0))
                .strafeTo(new Vector2d(62, -62))
                .build();

        delivery.clawClose();
        waitForStart();

        if (isStopRequested()) return;

        delivery.setSlidesTargetPosition(4000);
        while (Math.abs(delivery.getMotorTargetPosition() - delivery.getMotorPosition()) > 20) {
            delivery.PControlPower(2);
        }
        sleep(500);

        Actions.runBlocking(trajToSubmersible);

        delivery.setSlidesTargetPosition(3100);
        while (Math.abs(delivery.getMotorTargetPosition() - delivery.getMotorPosition()) > 20) {
            delivery.PControlPower(3);
        }
        delivery.clawOpen();
        sleep(1000);

        Actions.runBlocking(trajToCollectSamples);

        delivery.setSlidesTargetPosition(0);
        while (Math.abs(delivery.getMotorTargetPosition() - delivery.getMotorPosition()) > 20) {
            delivery.PControlPower(3);
        }
    }
}