package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class DeliveryFunctions {
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    private Servo wrist = null;
    private Servo holder1 = null;
    private Servo holder2 = null;
    private LinearOpMode linearOpMode;
    private double CLICKS_PER_METER = 2492.788;
    private double HOLDER_OPEN = 0;
    private double HOLDER_CLOSE = 1;
    private final double servoOut = 0.22;
    private final double servoIn = 0.16;

    private int targetPosition;
    private double currentPosition;

    private double servoPosition;
    private double targetServoPosition;

    public final double TICK_STOP_THRESHOLD = 20;
    public final double CARRIAGE_OUTSIDE_CHASSIS = 630;

    private boolean slidesRunToPosition;

    public boolean isDisabled = false;

    private double initAttempts = 0;


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
//            holder1 = linearOpMode.hardwareMap.get(Servo.class, "holder1");
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

            rightSlide.setDirection(DcMotor.Direction.REVERSE);
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
//            holder1 = linearOpMode.hardwareMap.get(Servo.class, "holder1");
//            holder2 = linearOpMode.hardwareMap.get(Servo.class, "holder2");
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

            rightSlide.setDirection(DcMotor.Direction.REVERSE);
            isDisabled = false;
        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
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

//    public void PControlPower(){
//        leftError = targetPosition - leftSlide.getCurrentPosition();
//        rightError = targetPosition - rightSlide.getCurrentPosition();
//
//        leftSlide.setPower(leftError / TICK_SLOW_THRESHOLD);
//        rightSlide.setPower(rightError / TICK_SLOW_THRESHOLD);
//    }

    public void Dump(){
        linearOpMode.telemetry.addLine("WIP");
    }

    public void WristMovementByLiftPosition(){
        servoPosition = wrist.getPosition();
        currentPosition = rightSlide.getCurrentPosition();
        if(currentPosition > CARRIAGE_OUTSIDE_CHASSIS){
            //targetServoPosition is going to be out when 200 ticks from outside
            targetServoPosition = servoOut; //* (currentPosition / CARRIAGE_OUTSIDE_CHASSIS + 200);
            wrist.setPosition(targetServoPosition);
        } else{
            wrist.setPosition(servoIn);
        }
        //.35
        //.08

        //Math.toRadians(100)
        //Math.toRadians(10)

        linearOpMode.telemetry.addData("wrist pos: ", servoPosition);
        linearOpMode.telemetry.update();

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
}
