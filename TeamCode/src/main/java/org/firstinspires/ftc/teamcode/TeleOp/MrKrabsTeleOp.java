package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.RoboMom;


@TeleOp(name="MrKrabs Teleop", group="AAA")
public class MrKrabsTeleOp extends RoboMom {

    private ElapsedTime deliveryTimer = new ElapsedTime();

    DrivetrainFunctions drivetrainFunctions = null;
    Delivery delivery = null;
    public enum DeliveryState{
        DELIVERY_START,
        DELIVERY_LIFT,
        DELIVERY_RETRACT
    }
    private DeliveryState deliveryState = DeliveryState.DELIVERY_START;
    private int slidePosition = 0;

    private int targetPosition = 0;

    private boolean controlsRelinquished = false;
    private final double DRIVE_DEADZONE = 0.05;
    private final double SCORE_SPEED_SCALAR = 0.2;

    private double speedScalar = 1;
    @Override
    public void runOpMode() {
        super.runOpMode();

        delivery = new Delivery(this, false);

        drivetrainFunctions = new DrivetrainFunctions(this);

        waitForStart();

        while (opModeIsActive()){
            //driver 1
            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                speedScalar = 0.5;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            } else {
                speedScalar = 1;
            }

            if (!controlsRelinquished) {

                if (Math.abs(gamepad1.left_stick_x) > DRIVE_DEADZONE || Math.abs(gamepad1.left_stick_y) > DRIVE_DEADZONE || Math.abs(gamepad1.right_stick_x) > DRIVE_DEADZONE || Math.abs(gamepad1.right_stick_y) > DRIVE_DEADZONE*2) {
                    if(Math.abs(gamepad1.right_stick_y) > DRIVE_DEADZONE*2) {
                        drivetrainFunctions.Move(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_x, speedScalar);
                    }else{
                        drivetrainFunctions.Move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speedScalar);
                    }
                } else {
                    drivetrainFunctions.Stop();
                }
            }

            //driver 2
            slidePosition = delivery.getMotorPosition();
            targetPosition = delivery.getMotorTargetPosition();


            switch (deliveryState) {
                case DELIVERY_START:
                    if (gamepad2.a) {
                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_LIFT;
                        targetPosition = 100;//SET_1_HEIGHT;
                    }
                    if (gamepad2.x) {
                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_LIFT;
                        targetPosition = 200;//SET_2_HEIGHT;
                    }
                    if (gamepad2.y) {
                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_LIFT;
                        targetPosition = 300;//SET_3_HEIGHT;
                    }
                    if (gamepad2.b) {
                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_RETRACT;
                        deliveryTimer.reset();
                    }

//                    if (gamepad2.right_bumper && !dumped && deliveryFunctions.getMotorPositionByIndex(0) > deliveryFunctions.CARRIAGE_OUTSIDE_CHASSIS) {
//                        deliveryState = DeliveryState.DELIVERY_DUMP;
//                        dumped = true;
//                        deliveryTimer.reset();
//                        deliveryFunctions.Dump(1);
//                        secondDumped = false;
//                    }
                    break;

                case DELIVERY_LIFT:
                    /*
                    //if both motors are within stop threshold
                    if
                    (targetPosition - leftMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD
                            &&
                            targetPosition - rightMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD) {
                        deliveryState = CenterStageTeleOp.DeliveryState.DELIVERY_START;
                        dumped = false;
                        secondDumped = false;
                    } else {
                        //Still going
                    }
                    */
                    break;



                case DELIVERY_RETRACT:
                    /*
                    retracting = true;
                    if(deliveryTimer.seconds() <= 0.5){
                        deliveryFunctions.SetWristPosition(deliveryFunctions.servoDodge);
                        break;
                    }else{
                        targetPosition = LIFT_LOW;
                        if (deliveryFunctions.getMotorPositionByIndex(0) < deliveryFunctions.CARRIAGE_DODGE) {
                            deliveryFunctions.SetWristPosition(deliveryFunctions.servoIn);
                        } else {
                            deliveryFunctions.SetWristPosition(deliveryFunctions.servoDodge);
                        }

                        //if both motors are within stop threshold
                        if
                        (leftMotorPosition - targetPosition <= deliveryFunctions.TICK_STOP_THRESHOLD
                                &&
                                rightMotorPosition - targetPosition <= deliveryFunctions.TICK_STOP_THRESHOLD) {

                            deliveryState = CenterStageTeleOp.DeliveryState.DELIVERY_START;
                            retracting = false;
                        }
                    }

                     */

                    break;

            }

            telemetry.addData("Delivery State: ", deliveryState);
            telemetry.addData("Delivery Timer: ", deliveryTimer.seconds());



            //MANUAL
            if (Math.abs(gamepad2.right_stick_y) >= DRIVE_DEADZONE || Math.abs(gamepad2.left_stick_y) >= DRIVE_DEADZONE) {
                deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_START;

                if (slidePosition > delivery.LOW_POSITION) {
                    targetPosition -= gamepad2.left_stick_y * 35;
                } else {
                    targetPosition -= gamepad2.left_stick_y * 25;
                }
            }

            targetPosition = Math.max(delivery.BOTTOM_POSITION, Math.min(delivery.TOP_POSITION, targetPosition));
            delivery.setSlidesTargetPosition(targetPosition);

            delivery.PControlPower();


            //telemetry.addData("Target Position: ", targetPosition);
            telemetry.addData("Target Position in DeliveryFunctions: ", delivery.getMotorTargetPosition());
            telemetry.addData("Left Motor Position: ", slidePosition);


        }
    }
}
