package org.firstinspires.ftc.teamcode.TeleOp;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
public class OneFishIntake {
    private DcMotor extension = null;
    private DcMotor intake = null;
    private Servo pitch = null;

    private LinearOpMode linearOpMode;

    public final double CLICKS_PER_DEGREE = 3.5;
    private final double CLICKS_PER_CM = 24.92788;
    private final int MM_PER_METER = 1000;

    public final int MIN_EXTENSION = 0;
    public final int MAX_EXTENSION = 1300;
    private final double MIN_PITCH = 0;
    private final double MAX_PITCH = 1;

    private final int TICK_LOW_POWER_DISTANCE = 50;

    private double targetLengthCM;
    private int targetLength;
    private double currentPosition;

    public final double TICK_STOP_THRESHOLD = 5;

    private ElapsedTime time = new ElapsedTime();



    public OneFishIntake(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){//4mm
        try {
            extension = linearOpMode.hardwareMap.get(DcMotor.class, "extension");
            extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intake = linearOpMode.hardwareMap.get(DcMotor.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//            pitch = linearOpMode.hardwareMap.get(Servo.class, "pitch");
//            pitch.scaleRange(0.0, 1.0);

//            pitchMotor.setDirection(DcMotorSimple.Direction.REVERSE);




        }catch(NullPointerException e){
            linearOpMode.telemetry.addLine("Couldn't find intake");
        }
    }


    public void setTargetLength(int ticks){
        targetLength = ticks;
        extension.setTargetPosition(Math.max(MIN_EXTENSION, Math.min(MAX_EXTENSION, ticks)));
    }

    public void setExtensionPower(double power){
        int error = 0;
        int errorMult = 0;
        int current = extension.getCurrentPosition();
        extension.setPower(power);

        if(current > MAX_EXTENSION/2){
            error = (current - MAX_EXTENSION);
        } else {
            error = (current-MIN_EXTENSION);
        }

        errorMult = error/(TICK_LOW_POWER_DISTANCE*2);

        if(Math.abs(error) < TICK_LOW_POWER_DISTANCE){
            extension.setPower(power * errorMult);
            linearOpMode.telemetry.addLine("BRAKING");
        }
        linearOpMode.telemetry.addData("Intake Current: ", current);
    }

    public void runToPosition(){
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runPower(){
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void updateLength(){
        linearOpMode.telemetry.addData("Intake Length: ", targetLength);
        linearOpMode.telemetry.addData("Intake Current: ", extension.getCurrentPosition());
        if(Math.abs(extension.getCurrentPosition() - extension.getTargetPosition()) > TICK_STOP_THRESHOLD){
            extension.setPower(1);
        } else{
            extension.setPower(0);
        }
    }

    public void setIntakePower(float power){
        intake.setPower(power);
    }

//    public void setPitch(float position){
//        pitch.setPosition(position);
//    }

}
