package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.OneFishIntake;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "AutoIntakeTest", group = "TEST")
public class AutoIntakeTest extends LinearOpMode {
    OneFishIntake intake = new OneFishIntake(this);


    public void runOpMode() {
        waitForStart();
        while(!opModeIsActive() || !isStopRequested()){
            intake.pitchToTransfer();
            sleep(1500);
            intake.pitchUp();
            sleep(1500);
            intake.pitchDown();
            sleep(1500);
        }
    }
}