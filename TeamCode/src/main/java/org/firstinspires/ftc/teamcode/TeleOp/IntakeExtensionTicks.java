package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.OneFishIntake;

@TeleOp(name="Intake Extension Ticks", group="TEST")
public class IntakeExtensionTicks extends LinearOpMode {
    private OneFishIntake intake;



    public void runOpMode() {
        intake = new OneFishIntake(this);

        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("Intake Extension: ", intake.getExtensionTicks());

            telemetry.update();

        }
    }
}
