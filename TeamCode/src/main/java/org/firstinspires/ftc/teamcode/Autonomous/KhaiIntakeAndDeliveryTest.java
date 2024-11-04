package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.Intake;

@Config
@Autonomous(name = "Khai Intake And Delivery Test", group = "Autonomous")
public class KhaiIntakeAndDeliveryTest extends LinearOpMode {

    Delivery delivery = null;
    Intake intake = null;

    @Override
    public void runOpMode() {
        delivery = new Delivery(this, false);
        intake = new Intake(this);

        waitForStart();

        sleep(500);
        intakeAndDeliveryToPosition(200, 3, -232);
        intake.updateAngle();
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
}
