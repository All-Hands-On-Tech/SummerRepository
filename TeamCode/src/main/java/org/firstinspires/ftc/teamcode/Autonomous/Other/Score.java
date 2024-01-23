package org.firstinspires.ftc.teamcode.Autonomous.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DeliveryFunctions;


@Autonomous(name="Score", group="Z")
public class Score extends LinearOpMode {

    DeliveryFunctions deliveryFunctions;
    @Override
    public void runOpMode() {

     deliveryFunctions = new DeliveryFunctions(this, true);

        waitForStart();
        if (isStopRequested()) return;

        deliveryFunctions.Score(100);
        sleep(5000);
        deliveryFunctions.Retract();

    }
}