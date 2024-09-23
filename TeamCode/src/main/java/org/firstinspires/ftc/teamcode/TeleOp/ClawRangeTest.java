package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Claw Range Test", group="TEST")
public class ClawRangeTest extends LinearOpMode {

    private Servo claw = null;
    private double pos = 0.0;
    private ElapsedTime timer;


    @Override
    public void runOpMode() {

        timer = new ElapsedTime();
        claw = hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(0.0, 0.25);

        waitForStart();
        while(opModeIsActive()){
            if(timer.seconds() >= 0.001){
                if(gamepad2.dpad_right)pos += 0.001;
                if(gamepad2.dpad_left)pos -= 0.001;
                timer.reset();
            }
//            pos = gamepad2.left_stick_x;
            pos = Math.min(pos, 1);
            pos = Math.max(pos, 0);
            claw.setPosition(pos);
            telemetry.addData("Servo Position", claw.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
