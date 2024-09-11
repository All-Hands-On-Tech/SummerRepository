package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DeliveryFunctions;

@Autonomous(name="AutomaticWristMovementTest", group="Z")
public class WristMovementByLiftPositionTEST extends LinearOpMode{

    DeliveryFunctions deliveryFunctions;

    int leftMotorPosition;
    int rightMotorPosition;

    @Override
    public void runOpMode() {
        deliveryFunctions = new DeliveryFunctions(this, false);

        waitForStart();

        while (opModeIsActive()) {
            deliveryFunctions.WristMovementByLiftPosition();

            leftMotorPosition = deliveryFunctions.getMotorPositionByIndex(0);
            rightMotorPosition = deliveryFunctions.getMotorPositionByIndex(1);

            telemetry.addData("Left Ticks:", leftMotorPosition);
            telemetry.addData("Right Ticks:", rightMotorPosition);
            telemetry.addData("Servo Position: ", deliveryFunctions.getWristPosition());
            telemetry.update();
        }
    }
}
