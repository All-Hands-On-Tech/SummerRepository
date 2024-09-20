package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
public class Delivery {
    private DcMotor slide = null;
    private Servo claw = null;


    private LinearOpMode linearOpMode;

    private double slidePowerMultiplier = 0.8;
    private double CLICKS_PER_METER = 2492.788;
    private final int MM_PER_METER = 1000;
    private int targetPosition;
    private double currentPosition;

    public final double TICK_STOP_THRESHOLD = 20;
    public final int BOTTOM_POSITION = (int)(2 / MM_PER_METER * CLICKS_PER_METER); //2mm
    public final int TOP_POSITION = 4300;
    public final int LOW_POSITION = 50; // low-to-ground position used to slow slides when low to ground
    public final double TICK_LOW_POWER_DISTANCE = 200;

    public final double RETRACT_TIMEOUT = 7;

    private boolean slidesRunToPosition;

    public boolean isDisabled = false;

    private double initAttempts = 0;

    NormalizedColorSensor sensorFront;
    NormalizedRGBA colorsFront;
    final float[] HSVValuesFront = new float[3];
    NormalizedColorSensor sensorBack;
    NormalizedRGBA colorsBack;
    final float[] HSVValuesBack = new float[3];

    double yellow = 86;
    double green = 125;
    double white = 155;
    double purple = 206;
    double distance = 1.8;

    private ElapsedTime time = new ElapsedTime();



    public Delivery(LinearOpMode l, Boolean slidesRunToPosition)
    {
        linearOpMode = l;
        slidesRunToPosition = slidesRunToPosition;
        Initialize();
    }


    private void Initialize(){//4mm
        try {
            slide  = linearOpMode.hardwareMap.get(DcMotor.class, "deliverySlide");
            claw = linearOpMode.hardwareMap.get(Servo.class, "claw");
            claw.scaleRange(0.4, 0.5);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slide.setTargetPosition(slide.getCurrentPosition());

            if (slidesRunToPosition) {
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            slide.setDirection(DcMotor.Direction.REVERSE);

        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void Reinitialize(){//4mm
        try {
            slide  = linearOpMode.hardwareMap.get(DcMotor.class, "deliverySlide");
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slide.setTargetPosition(slide.getCurrentPosition());

            if (slidesRunToPosition) {
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            slide.setDirection(DcMotor.Direction.REVERSE);

        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void setSlidesTargetPosition(int clicks){
        targetPosition = clicks;
        slide.setTargetPosition(targetPosition);
    }

    public void setSlidesPower(double power){
        slide.setPower(power);
    }

    public int getMotorPosition(){ return slide.getCurrentPosition(); }

    public int getMotorTargetPosition(){
        return targetPosition;
    }

    public DcMotor.RunMode getRunMode(){
        return slide.getMode();
    }
    public void setRunMode(DcMotor.RunMode mode){ slide.setMode(mode); }

    public void PControlPower(){
        double error = targetPosition - slide.getCurrentPosition();
        double power = (Math.abs(error) / TICK_LOW_POWER_DISTANCE);

        power = Math.max(0, Math.min(1, power));

        slide.setPower(power * slidePowerMultiplier);
        linearOpMode.telemetry.addData("Power: ",power);
        linearOpMode.telemetry.addData("Error: ",error);
        linearOpMode.telemetry.update();
    }

    public void setClawPosition(double p){ // 0-1
        claw.setPosition(p);
    }
    public void clawOpen(){
        setClawPosition(1);
    }
    public void clawClose(){
        setClawPosition(0);
    }
/*

    public void Lift(int ticksFromOutsideChassis){
        slidePowerMultiplier = 0.75;
        setSlidesTargetPosition(BOTTOM_POSITION);
        PControlPower();

        double error = targetPosition - slide.getCurrentPosition();
        error = Math.abs(error);

        //LIFT
        while(error <= TICK_STOP_THRESHOLD){

            error = targetPosition - slide.getCurrentPosition();
            error = Math.abs(error);

            PControlPower();
            linearOpMode.telemetry.addData("Error: ", error);
            linearOpMode.telemetry.update();
        }
        linearOpMode.sleep(1000);

        setSlidesTargetPosition(CARRIAGE_OUTSIDE_CHASSIS + ticksFromOutsideChassis - 150);
        while(leftError <= TICK_STOP_THRESHOLD
                &&
                rightError <= TICK_STOP_THRESHOLD){

            leftError = targetPosition - leftSlide.getCurrentPosition();
            rightError = targetPosition - rightSlide.getCurrentPosition();
            leftError = Math.abs(leftError);
            rightError = Math.abs(rightError);

            PControlPower();
            linearOpMode.telemetry.addData("Left Error: ", leftError);
            linearOpMode.telemetry.addData("right Error: ", rightError);
            linearOpMode.telemetry.update();
        }
    }


    public void Retract(){
        time.reset();
        wrist.setPosition(servoDodge);

        //Waiting for servo to move
        while(time.seconds() < 0.5){
            linearOpMode.telemetry.addLine("Waiting to Retract");
            linearOpMode.telemetry.update();
        }
        int tempTarget = leftSlide.getCurrentPosition();
        while(((leftSlide.getCurrentPosition() > 0 || rightSlide.getCurrentPosition() > 0) || time.seconds() > RETRACT_TIMEOUT) && tempTarget > CARRIAGE_DODGE){
            tempTarget -= 10;

            setSlidesTargetPosition(tempTarget);

            setSlidesPower(0.5);
        }
//        tempTarget = leftSlide.getCurrentPosition();
        setSlidesTargetPosition(tempTarget);
        while(leftSlide.getCurrentPosition() > CARRIAGE_DODGE){

        }
        time.reset();
        while(time.seconds() < 0.3){
            linearOpMode.telemetry.addLine("Waiting to Fully Retract");
            linearOpMode.telemetry.update();
        }
        wrist.setPosition(servoIn);
        time.reset();
        while(time.seconds() < 0.25){
            linearOpMode.telemetry.addLine("Waiting to Fully Retract");
            linearOpMode.telemetry.update();
        }
        tempTarget = leftSlide.getCurrentPosition();
        //RETRACT
        while((leftSlide.getCurrentPosition() > 0 || rightSlide.getCurrentPosition() > 0) || time.seconds() > RETRACT_TIMEOUT){
            tempTarget -= 5;

            setSlidesTargetPosition(tempTarget);

            setSlidesPower(0.25);

            if(leftSlide.getCurrentPosition() < CARRIAGE_DODGE){
                wrist.setPosition(servoIn);
            }
        }
        setSlidesPower(0);



        slidePowerMultiplier = 0.75;
    }
    */
}
