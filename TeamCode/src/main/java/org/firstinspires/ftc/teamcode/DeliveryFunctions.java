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
public class DeliveryFunctions {
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    private Servo wrist = null;
    private Servo holder1 = null;
    private Servo holder2 = null;

    private LinearOpMode linearOpMode;

    private double slidePowerMultiplier = 0.8;
    private double CLICKS_PER_METER = 2492.788;
    private double HOLDER_OPEN = 0;
    private double HOLDER_CLOSE = 1;

    public final double servoIn = 0.2425;
    private final double servoOut = 0.51;//0.77

    public final double servoDodge = 0.225;//0.421

    private int targetPosition;
    private double currentPosition;

    private double servoPosition;
    private double targetServoPosition;

    public final double TICK_STOP_THRESHOLD = 20;
    public final int CARRIAGE_OUTSIDE_CHASSIS = 740;
    public final double TICK_LOW_POWER_DISTANCE = 200;
    public final double CARRIAGE_DODGE = 150;

    public final double DUMP_TIME = 0.5;

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
    double white = 173;
    double purple = 206;
    double distance = 1.8;

    private ElapsedTime time = new ElapsedTime();


    public DeliveryFunctions(LinearOpMode l, Boolean slidesRunToPosition)
    {
        linearOpMode = l;
        slidesRunToPosition = slidesRunToPosition;
        Initialize();
    }


    private void Initialize(){
        try {
            leftSlide  = linearOpMode.hardwareMap.get(DcMotor.class, "leftSlide");
            rightSlide  = linearOpMode.hardwareMap.get(DcMotor.class, "rightSlide");
            wrist = linearOpMode.hardwareMap.get(Servo.class, "wrist");
            holder1 = linearOpMode.hardwareMap.get(Servo.class, "holder1");
            holder2 = linearOpMode.hardwareMap.get(Servo.class, "holder2");
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setTargetPosition(leftSlide.getCurrentPosition());
            rightSlide.setTargetPosition(rightSlide.getCurrentPosition());

            if (slidesRunToPosition) {
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                linearOpMode.telemetry.addLine("RunMode: RUN_TO_POSITION");
//                linearOpMode.telemetry.update();
            } else {
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//            rightSlide.setDirection(DcMotor.Direction.REVERSE);

            sensorFront = linearOpMode.hardwareMap.get(NormalizedColorSensor.class, "front_color");
            sensorFront.setGain(10);
            sensorBack = linearOpMode.hardwareMap.get(NormalizedColorSensor.class, "back_color");
            sensorBack.setGain(10);

            leftSlide.setDirection(DcMotor.Direction.REVERSE);

        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void Reinitialize(){
        try {
            leftSlide  = linearOpMode.hardwareMap.get(DcMotor.class, "leftSlide");
            rightSlide  = linearOpMode.hardwareMap.get(DcMotor.class, "rightSlide");
            wrist = linearOpMode.hardwareMap.get(Servo.class, "wrist");
            holder1 = linearOpMode.hardwareMap.get(Servo.class, "holder1");
            holder2 = linearOpMode.hardwareMap.get(Servo.class, "holder2");
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setTargetPosition(leftSlide.getCurrentPosition());
            rightSlide.setTargetPosition(rightSlide.getCurrentPosition());

            if (slidesRunToPosition) {
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearOpMode.telemetry.addLine("RunMode: RUN_TO_POSITION");
                linearOpMode.telemetry.update();
            } else {
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            rightSlide.setDirection(DcMotor.Direction.REVERSE);

            sensorFront = linearOpMode.hardwareMap.get(NormalizedColorSensor.class, "front_color");
            sensorFront.setGain(10);
            sensorBack = linearOpMode.hardwareMap.get(NormalizedColorSensor.class, "back_color");
            sensorBack.setGain(10);

            leftSlide.setDirection(DcMotor.Direction.REVERSE);

            isDisabled = false;
        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public String detectFrontPixelColor() {
        colorsFront = sensorFront.getNormalizedColors();
        Color.colorToHSV(colorsFront.toColor(), HSVValuesFront);
        if (((DistanceSensor) sensorFront).getDistance(DistanceUnit.CM) < distance) {
            if (HSVValuesFront[0] > yellow-10 && HSVValuesFront[0] < yellow+10) {
                return "YELLOW";
            } else if (HSVValuesFront[0] > green-10 && HSVValuesFront[0] < green+10) {
                return "GREEN";
            } else if (HSVValuesFront[0] > white-10 && HSVValuesFront[0] < white+10) {
                return "WHITE";
            } else if (HSVValuesFront[0] > purple-10 && HSVValuesFront[0] < purple+10) {
                return "PURPLE";
            }
        }
        return "NONE";
    }

    public String detectBackPixelColor() {
        colorsBack = sensorBack.getNormalizedColors();
        Color.colorToHSV(colorsBack.toColor(), HSVValuesBack);
        if (((DistanceSensor) sensorBack).getDistance(DistanceUnit.CM) < distance) {
            if (HSVValuesBack[0] > yellow-10 && HSVValuesBack[0] < yellow+10) {
                return "YELLOW";
            } else if (HSVValuesBack[0] > green-10 && HSVValuesBack[0] < green+10) {
                return "GREEN";
            } else if (HSVValuesBack[0] > white-10 && HSVValuesBack[0] < white+10) {
                return "WHITE";
            } else if (HSVValuesBack[0] > purple-10 && HSVValuesBack[0] < purple+10) {
                return "PURPLE";
            }
        }
        return "NONE";
    }

    public void setSlidesTargetPosition(int clicks){
        targetPosition = clicks;
        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);
    }

    public void setSlidesPower(double power){
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public int getMotorPositionByIndex(int index){
        if(index == 0){
            return leftSlide.getCurrentPosition();
        } else {
            return rightSlide.getCurrentPosition();
        }

    }

    public int getMotorTargetPosition(){
        return targetPosition;
    }

    public DcMotor.RunMode getRunMode(){
        return leftSlide.getMode();
    }
    public void setRunMode(DcMotor.RunMode mode){
        leftSlide.setMode(mode);
        rightSlide.setMode(mode);
    }

    public double getWristPosition(){
        return wrist.getPosition();
    }

    public void PControlPower(){
        double leftError = targetPosition - leftSlide.getCurrentPosition();
        double rightError = targetPosition - rightSlide.getCurrentPosition();

        double leftPower = (Math.abs(leftError) / TICK_LOW_POWER_DISTANCE);
        double rightPower = (Math.abs(rightError) / TICK_LOW_POWER_DISTANCE);

        leftPower = Math.max(0, Math.min(1, leftPower));
        rightPower = Math.max(0, Math.min(1, rightPower));

        leftSlide.setPower(leftPower * slidePowerMultiplier);
        rightSlide.setPower(rightPower * slidePowerMultiplier);
        linearOpMode.telemetry.addData("Left Power: ",leftPower);
        linearOpMode.telemetry.addData("Right Power: ",rightPower);
        linearOpMode.telemetry.addData("Left Error: ",leftError);
        linearOpMode.telemetry.addData("Right Error: ",rightError);
        linearOpMode.telemetry.update();
    }

    public void Dump(int holder){
        if(holder == 0){
            //OPEN BOTH AUTO
            time.reset();
            //LIFT SLIDES
            while(time.seconds() < DUMP_TIME){
                holder1.setPosition(HOLDER_OPEN);
                holder2.setPosition(HOLDER_OPEN);

                if(time.seconds() > DUMP_TIME/2){
                    setSlidesTargetPosition(leftSlide.getCurrentPosition()+300);
                }
            }

            holder1.setPosition(HOLDER_CLOSE);
            holder2.setPosition(HOLDER_CLOSE);
        }

        if(holder == 1){
            //OPEN 1
            time.reset();
            while(time.seconds() < DUMP_TIME){
                holder1.setPosition(HOLDER_OPEN);
            }
            //CLOSE 1
            time.reset();
            while(time.seconds() < DUMP_TIME){
                holder1.setPosition(HOLDER_CLOSE);
            }
            //OPEN 2
            time.reset();
            while(time.seconds() < DUMP_TIME){
                holder2.setPosition(HOLDER_OPEN);
            }
        }
        if(holder == 2){
            //OPEN 2
            time.reset();
            while(time.seconds() < DUMP_TIME){
                holder2.setPosition(HOLDER_OPEN);
            }
            holder2.setPosition(HOLDER_CLOSE);
        }
        if(holder == 3){
            //OPEN BOTH
            time.reset();
            while(time.seconds() < DUMP_TIME){
                holder1.setPosition(HOLDER_OPEN);
                holder2.setPosition(HOLDER_OPEN);
            }
            holder1.setPosition(HOLDER_CLOSE);
            holder2.setPosition(HOLDER_CLOSE);
        }


    }

    public void Score(int ticksFromOutsideChassis){
        slidePowerMultiplier = 0.75;
        setSlidesTargetPosition(CARRIAGE_OUTSIDE_CHASSIS + ticksFromOutsideChassis);
        PControlPower();

        double leftError = targetPosition - leftSlide.getCurrentPosition();
        double rightError = targetPosition - rightSlide.getCurrentPosition();
        leftError = Math.abs(leftError);
        rightError = Math.abs(rightError);
        //LIFT
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
        linearOpMode.sleep(750);

        wrist.setPosition(servoOut);

        linearOpMode.sleep(1000);

        setSlidesTargetPosition(CARRIAGE_OUTSIDE_CHASSIS + ticksFromOutsideChassis - 50);
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

        linearOpMode.sleep(1000);
        //DUMP
        Dump(0);

        wrist.setPosition(servoDodge);
    }

    public void Retract(){
        time.reset();
        wrist.setPosition(servoDodge);

        //Waiting for servo to move
        while(time.seconds() > 1){
            linearOpMode.telemetry.addLine("Waiting to Retract");
            linearOpMode.telemetry.update();
        }

        int tempTarget = leftSlide.getCurrentPosition();
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

    public void WristMovementByLiftPosition(){
        servoPosition = wrist.getPosition();
        currentPosition = rightSlide.getCurrentPosition();
        if(currentPosition > CARRIAGE_OUTSIDE_CHASSIS - 150){
            //targetServoPosition is going to be out when 200 ticks from outside
            targetServoPosition = servoOut; //* (currentPosition / CARRIAGE_OUTSIDE_CHASSIS + 200);
            wrist.setPosition(targetServoPosition);
        } else{
            if(currentPosition > CARRIAGE_DODGE){
                wrist.setPosition(servoDodge);
            }else{
                wrist.setPosition(servoIn);
            }
        }
        //.35
        //.08

        //Math.toRadians(100)
        //Math.toRadians(10)

//        linearOpMode.telemetry.addData("wrist pos: ", servoPosition);
//        linearOpMode.telemetry.update();

    }

    public void OpenHolderServoByIndex(int i){
        if(i == 0){
            holder1.setPosition(HOLDER_OPEN);
        }
        if(i == 1){
            holder2.setPosition(1);
        }
    }

    public void CloseHolderServoByIndex(int i){
        if(i == 0){
            holder1.setPosition(HOLDER_CLOSE);
        }
        if(i == 1){
            holder2.setPosition(0);
        }
    }

    public void SetWristPosition(double position){
        wrist.setPosition(position);
    }
}
