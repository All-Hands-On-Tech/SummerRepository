package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.IntakeFunctions;

@TeleOp(name="Arm Pitch Test", group="TEST")
public class ArmPitchTest extends LinearOpMode {
    private Intake intake;



    public void runOpMode() {
        intake = new Intake(this);

        waitForStart();

        while(opModeIsActive()){

            intake.setTargetAngle(intake.getPitchMotorPosition() / intake.CLICKS_PER_DEGREE);
            intake.setTargetLength(30);

            telemetry.addLine("Assumed arm length = 30 cm");
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addData("X:", intake.getCartesianTargetXFromPolar());
            telemetry.addData("Y:", intake.getCartesianTargetYFromPolar());
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addData("L:", 30);
            telemetry.addData("theta:", intake.getTargetAngle());

            telemetry.update();

        }
    }
}
