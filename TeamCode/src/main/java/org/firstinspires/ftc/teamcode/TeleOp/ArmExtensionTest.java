package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.IntakeFunctions;

@TeleOp(name="Arm Extension Test", group="TEST")
public class ArmExtensionTest extends LinearOpMode {
    private Intake intake;



    public void runOpMode() {
        intake = new Intake(this);

        waitForStart();

        while(opModeIsActive()){

            intake.setTargetAngle(intake.getPitchMotorPosition() / intake.CLICKS_PER_DEGREE);
            if(gamepad2.dpad_up) {
                intake.setTargetLength(60);
            }
            if(gamepad2.dpad_right) {
                intake.setTargetLength(50);
            }
            if(gamepad2.dpad_left) {
                intake.setTargetLength(40);
            }
            if(gamepad2.dpad_down) {
                intake.setTargetLength(30);
            }

            double extension = 0.29;
            float rightTrigger = gamepad2.right_trigger;
            float leftTrigger = gamepad2.left_trigger;
            if(rightTrigger > 0){
                extension -= rightTrigger/100;
                extension = Math.max(0.01, Math.min(0.25, extension));
                intake.setTargetLengthServo(extension);
            } else if(leftTrigger > 0){
                extension += leftTrigger/100;
                extension = Math.max(0.01, Math.min(0.25, extension));
                intake.setTargetLengthServo(extension);
            }

            intake.updateLength();

            telemetry.addLine("dpad_up = 60 cm, dpad_right = 50 cm, dpad_left = 40 cm, dpad_down = 30 cm");
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addData("X:", intake.getCartesianTargetXFromPolar());
            telemetry.addData("Y:", intake.getCartesianTargetYFromPolar());
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine("L: ");
            telemetry.addData("theta:", intake.getTargetAngle());
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addData("Intake Extension: ", intake.getExtensionServoPosition());

            telemetry.update();

        }
    }
}
