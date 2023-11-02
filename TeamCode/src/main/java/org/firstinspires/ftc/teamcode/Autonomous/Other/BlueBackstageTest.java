package org.firstinspires.ftc.teamcode.Autonomous.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpmode;
@Autonomous(name = "TEST AUTO")
public class BlueBackstageTest extends AutonomousOpmode {
    @Override
    public void runOpMode() {

        isRed = false;
        isBackstage = true;
        super.runOpMode();
    }
}
