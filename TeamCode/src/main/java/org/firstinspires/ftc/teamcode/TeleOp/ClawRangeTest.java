package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="Claw Range Test", group="TEST")
public class ClawRangeTest extends LinearOpMode {

    private Servo claw = null;
    private double pos = 0.0;


    @Override
    public void runOpMode() {

        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();
        while(opModeIsActive()){
            pos = gamepad2.left_stick_x;
            pos = Math.min(pos, 1);
            pos = Math.max(pos, 0);
            telemetry.addData("Claw Position:", pos);
            claw.setPosition(pos);
            telemetry.update();
        }
    }
}
