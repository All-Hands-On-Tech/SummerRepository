package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class IntakeFunctions {
    private DcMotor intakeMotor = null;
    private LinearOpMode linearOpMode;
    private double CLICKS_PER_METER = 2492.788;

    public IntakeFunctions(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){
        intakeMotor  = linearOpMode.hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void RunIntakeMotor(float power){
        intakeMotor.setPower(power);
    }

    public void StopIntakeMotor(){
        intakeMotor.setPower(0);
    }

}
