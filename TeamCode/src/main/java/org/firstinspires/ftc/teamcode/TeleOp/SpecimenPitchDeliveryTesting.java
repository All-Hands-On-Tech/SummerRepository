package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.OneFishSpecimenDelivery;

@TeleOp(name="SpecimenPitchDeliveryTesting", group="TEST")
public class SpecimenPitchDeliveryTesting extends LinearOpMode {
    private OneFishSpecimenDelivery specimenDelivery;

    private double target = 0.0;
    private double clawTarget = 0.0;


    public void runOpMode() {
        specimenDelivery = new OneFishSpecimenDelivery(this);

        target = specimenDelivery.DELIVERY_PITCH;
        clawTarget = specimenDelivery.CLAW_CLOSE;

        waitForStart();

        while(opModeIsActive()){

            if(gamepad2.dpad_up){
                target += 0.01;
            }
            if(gamepad2.dpad_down){
                target -= 0.01;
            }

            if(gamepad2.dpad_right){
                clawTarget += 0.01;
            }
            if(gamepad2.dpad_left){
                clawTarget -= 0.01;
            }

            specimenDelivery.setPivotPosition(target);
            specimenDelivery.setClawPosition(clawTarget);

            telemetry.addData("Pivot Target: ", target);
            telemetry.addLine();
            telemetry.addData("Claw Target: ", clawTarget);

            telemetry.update();

        }
    }
}
