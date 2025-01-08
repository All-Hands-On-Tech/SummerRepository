package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.OneFishIntake;
import org.firstinspires.ftc.teamcode.OneFishSampleDelivery;
import org.firstinspires.ftc.teamcode.OneFishSpecimenDelivery;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Auto Action Testing", group = "TEST")
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

        waitForStart();

        if (isStopRequested()) return;

            Actions.runBlocking(driveTrain.TurnToAngleAction(Math.toRadians(-45), 1000));
            telemetry.addData("Heading: ", driveTrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
    }
}