package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DeliveryFunctions;

@Autonomous(name="AutomaticWristMovementTest", group="Z")
public class WristMovementByLiftPositionTEST extends LinearOpMode{

    DeliveryFunctions DeliveryFunctions;

    DcMotor leftMotorPosition;
    DcMotor rightMotorPosition;

    @Override
    public void runOpMode() {
        DeliveryFunctions = new DeliveryFunctions(this);

        waitForStart();

        while (opModeIsActive()) {
            DeliveryFunctions.WristMovementByLiftPosition();

            leftMotorPosition = deliveryFunctions.getMotorPositionByIndex(0);
            rightMotorPosition = deliveryFunctions.getMotorPositionByIndex(1);

            telemetry.addData("Left Ticks:", leftMotorPosition);
            telemetry.addData("Right Ticks:", rightMotorPosition);
        }
    }
}
