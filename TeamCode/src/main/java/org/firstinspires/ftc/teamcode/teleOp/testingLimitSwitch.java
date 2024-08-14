package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Limit Switch Experiment", group="A")
public class testingLimitSwitch extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor slideMotor = null;
    TouchSensor touchSensor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        slideMotor = hardwareMap.get(DcMotor.class, "motor");
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        boolean stop = false;
        double slidePower = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (!stop) {
             stop = touchSensor.isPressed();
            }
            boolean override = gamepad1.a;

            if (stop && !override) {
                slidePower = 0;
            } else {
                slidePower = gamepad1.left_stick_y / 3;
            }


            telemetry.addData("Motor Power", "(%1.2f)", slidePower);
            telemetry.addData("100 Counts", slideMotor.getCurrentPosition());
            if (touchSensor.isPressed()) {
                telemetry.addLine("Touch Sensor Is Pressed");
            } else {
                telemetry.addLine("Touch Sensor Is Not Pressed");
            }
            telemetry.update();

        }
    }
}
