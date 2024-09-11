package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class RumbleEffects {
    public Gamepad.RumbleEffect EndgameRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 0, 200)
            .addStep(0, 1, 200)
            .addStep(1, 0, 200)
            .addStep(0, 1, 200)
            .addStep(1, 0, 200)
            .addStep(0, 1, 200)
            .build();

    public Gamepad.RumbleEffect DumpedRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 2000)
            .build();

}
