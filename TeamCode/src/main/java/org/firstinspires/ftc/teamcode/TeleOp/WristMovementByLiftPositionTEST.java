package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DeliveryFunctions;

@Autonomous(name="AutomaticWristMovementTest", group="Z")
public class WristMovementByLiftPositionTEST extends LinearOpMode{

    DeliveryFunctions DeliveryFunctions;

    @Override
    public void runOpMode() {
        DeliveryFunctions = new DeliveryFunctions(this);

        waitForStart();

        while (opModeIsActive()) {
            DeliveryFunctions.WristMovementByLiftPosition();
        }
    }
}
