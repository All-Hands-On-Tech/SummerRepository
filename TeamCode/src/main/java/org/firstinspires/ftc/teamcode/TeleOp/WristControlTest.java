package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DeliveryFunctions;

@Autonomous(name="MANUAL WRIST CONTROL", group="Z")
public class WristControlTest extends LinearOpMode{

    DeliveryFunctions deliveryFunctions;

    int leftMotorPosition;
    int rightMotorPosition;
    double targetServoPosition = 0.5;

    @Override
    public void runOpMode() {
        deliveryFunctions = new DeliveryFunctions(this, false);

        waitForStart();

        while (opModeIsActive()) {
//            deliveryFunctions.WristMovementByLiftPosition();

            if(gamepad2.dpad_up){
                targetServoPosition += 0.0005;
            }
            if(gamepad2.dpad_down){
                targetServoPosition -= 0.0005;
            }

            deliveryFunctions.SetWristPosition(targetServoPosition);

            telemetry.addData("Servo Position: ", deliveryFunctions.getWristPosition());
            telemetry.addData("Servo Position: ", targetServoPosition);
            telemetry.update();
        }
    }
}
