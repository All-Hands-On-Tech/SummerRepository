package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    private double slidePowerMultiplier = 0.75;
    private double CLICKS_PER_METER = 2492.788;
    private double HOLDER_OPEN = 0;
    private double HOLDER_CLOSE = 1;

    private final double servoIn = 0.438;
    private final double servoOut = servoIn + 0.332;//0.77

    private final double servoDodge = servoIn - 0.017;//0.421

    private int targetPosition;
    private double currentPosition;

    private double servoPosition;
    private double targetServoPosition;

    public final double TICK_STOP_THRESHOLD = 20;
    public final int CARRIAGE_OUTSIDE_CHASSIS = 640;
    public final double TICK_LOW_POWER_DISTANCE = 200;
    public final double CARRIAGE_DODGE = 135;

    public final double DUMP_TIME = 1.5;

    private boolean slidesRunToPosition;

    public boolean isDisabled = false;

    private double initAttempts = 0;

    private colorSensor frontColorSensor;
    private colorSensor backColorSensor;

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

//            frontColorSensor.InitializeColorSensor("front_color");
//            backColorSensor.InitializeColorSensor("back_color");

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

//            frontColorSensor.InitializeColorSensor("front_color");
//            backColorSensor.InitializeColorSensor("back_color");

            leftSlide.setDirection(DcMotor.Direction.REVERSE);

            isDisabled = false;
        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    class colorSensor {
        NormalizedColorSensor sensor;
        String backColor = "NONE";

        NormalizedRGBA colors;

        final float[] HSVValues = new float[3];

        void InitializeColorSensor(String deviceName) {
            sensor = linearOpMode.hardwareMap.get(NormalizedColorSensor.class, deviceName);
            sensor.setGain(10);
        }

        void updateColorSensor() {
            colors = sensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), HSVValues);
        }

        public String detectPixelColor() {
            updateColorSensor();
            if (((DistanceSensor) sensor).getDistance(DistanceUnit.CM) < 6.5) {
                if (HSVValues[0] > 80 && HSVValues[0] < 100) {
                    return "YELLOW";
                } else if (HSVValues[0] > 120 && HSVValues[0] < 150) {
                    return "GREEN";
                } else if (HSVValues[0] > 180 && HSVValues[0] < 200) {
                    return "WHITE";
                } else if (HSVValues[0] > 210 && HSVValues[0] < 230) {
                    return "PURPLE";
                }
            }
            return "NONE";
        }
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

    public void Dump(){
        time.reset();
        while(time.seconds() < DUMP_TIME){
            holder1.setPosition(HOLDER_OPEN);
        }
            holder1.setPosition(HOLDER_CLOSE);
    }

    public void Score(){

        setSlidesTargetPosition(CARRIAGE_OUTSIDE_CHASSIS);
        PControlPower();

        double leftError = targetPosition - leftSlide.getCurrentPosition();
        double rightError = targetPosition - rightSlide.getCurrentPosition();
        leftError = Math.abs(leftError);
        rightError = Math.abs(rightError);

        wrist.setPosition(servoOut);
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

        linearOpMode.sleep(1000);
        //DUMP
        Dump();
        setSlidesTargetPosition(CARRIAGE_OUTSIDE_CHASSIS + 300);
        setSlidesPower(0.5);
        wrist.setPosition(servoDodge);
        linearOpMode.sleep(500);

        setSlidesTargetPosition(0);
        //RETRACT
        while(leftError > 0
                &&
                rightError > 0){

            leftError = targetPosition - leftSlide.getCurrentPosition();
            rightError = targetPosition - rightSlide.getCurrentPosition();
            leftError = Math.abs(leftError);
            rightError = Math.abs(rightError);

            PControlPower();
        }
    }

    public void WristMovementByLiftPosition(){
        servoPosition = wrist.getPosition();
        currentPosition = rightSlide.getCurrentPosition();
        if(currentPosition > CARRIAGE_OUTSIDE_CHASSIS){
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
            holder2.setPosition(HOLDER_OPEN);
        }
    }

    public void CloseHolderServoByIndex(int i){
        if(i == 0){
            holder1.setPosition(HOLDER_CLOSE);
        }
        if(i == 1){
            holder2.setPosition(HOLDER_CLOSE);
        }
    }

    public void SetWristPosition(double position){
        wrist.setPosition(position);
    }
}
