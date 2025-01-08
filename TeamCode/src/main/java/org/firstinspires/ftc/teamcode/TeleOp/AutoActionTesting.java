package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.OneFishIntake;
import org.firstinspires.ftc.teamcode.OneFishSampleDelivery;
import org.firstinspires.ftc.teamcode.OneFishSpecimenDelivery;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@TeleOp(name = "Auto Action Teleop", group = "TEST")
public class AutoActionTesting extends LinearOpMode {
    OneFishSpecimenDelivery specimenDelivery = null;
    OneFishIntake intake = null;
    OneFishSampleDelivery sampleDelivery = null;

    DrivetrainFunctions driveTrain = null;



    private boolean imuInitialized = false;

    @Override
    public void runOpMode() {
        driveTrain = new DrivetrainFunctions(this, true);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(15.2, -62, Math.toRadians(90)));


        specimenDelivery = new OneFishSpecimenDelivery(this);
        intake = new OneFishIntake(this);
        sampleDelivery = new OneFishSampleDelivery(this, true);


        imuInitialized = driveTrain.imu.initialize(driveTrain.myIMUparameters);

        telemetry.addData("Heading: ", driveTrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {
            if(gamepad1.a) {
                Actions.runBlocking(driveTrain.TurnToAngleAction(Math.toRadians(45), 1000));
            }
            if(gamepad1.b) {
                Actions.runBlocking(driveTrain.TurnToAngleAction(Math.toRadians(-45), 1000));
            }

            telemetry.addData("Heading: ", driveTrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

    }
}