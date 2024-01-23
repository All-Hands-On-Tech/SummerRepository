package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class IntakeFunctions {
    private DcMotor intakeMotor = null;
    private LinearOpMode linearOpMode;
    private double CLICKS_PER_METER = 2492.788;

    public boolean isDisabled = false;

    private double initAttempts = 0;

    private ElapsedTime time = new ElapsedTime();

    public float outPower = 0.175f;

    public IntakeFunctions(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){
//        try {
            intakeMotor = linearOpMode.hardwareMap.get(DcMotor.class, "intake");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        }catch(NullPointerException e){
//            initAttempts++;
//            linearOpMode.telemetry.addData("Couldn't find intake.       Attempt: ", initAttempts);
//            isDisabled = true;
//        }
    }

    public void Reinitialize(){
        try {
            intakeMotor = linearOpMode.hardwareMap.get(DcMotor.class, "intake");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            isDisabled = false;
        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find intake.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void RunIntakeMotor(float power){
        if(isDisabled)
            return;

        intakeMotor.setPower(power);
    }

    public void StopIntakeMotor(){
        if(isDisabled)
            return;

        intakeMotor.setPower(0);
    }

    public void OutakeFromIntake(float power){
        if(isDisabled)
            return;

        intakeMotor.setPower(power);
    }

    public void OutakeFromIntakeForTime(float power, double outTime){
        time.reset();
        if(isDisabled)
            return;
        while (outTime > time.seconds()) {
            intakeMotor.setPower(power);
        }
        intakeMotor.setPower(0);
    }

}
