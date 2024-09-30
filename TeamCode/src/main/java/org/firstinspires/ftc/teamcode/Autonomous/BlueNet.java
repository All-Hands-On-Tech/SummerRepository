package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Blue Net", group = "Autonomous")
public class BlueNet extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-14, -62, Math.toRadians(90)));

        Action trajectoryAction1;
        Action trajectoryAction2;

        //FIXME:    SPLIT trajectoryAction1 INTO MULTIPLE ACTIONS SO DELIVERY ACTIONS CAN BE IMPLEMENTED IN PARALLEL / SEQUENTIALLY

        trajectoryAction1 = drive.actionBuilder(drive.pose)

                .strafeTo(new Vector2d(-10, -34))
//               //score specimen
                .strafeTo(new Vector2d(-26, -40))
                .setTangent(110)
                .splineToLinearHeading(new Pose2d(-45, -14, Math.toRadians(90)), Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(-58, -58, Math.toRadians(45)), Math.toRadians(225))
                .strafeTo(new Vector2d( -40, -27))
                .splineToLinearHeading(new Pose2d(-57.00, -14, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-58, -58, Math.toRadians(60)), Math.toRadians(240))
                .strafeTo(new Vector2d( -50, -27))
                .splineToLinearHeading(new Pose2d(-62.00, -24, Math.toRadians(0)), Math.toRadians(180))
                .strafeTo(new Vector2d(-62, -55))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction1
                )
        );
    }
}