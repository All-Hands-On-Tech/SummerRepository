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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double slidePower = gamepad1.left_stick_y;

            slideMotor.setPower(slidePower);

            telemetry.addData("Motor Power", "(%.2f)", slidePower);
            telemetry.addData("1000 Counts", "(%2.3f)", slideMotor.getCurrentPosition()/1000);
            if (touchSensor.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }
            telemetry.update();
        }
    }
}
