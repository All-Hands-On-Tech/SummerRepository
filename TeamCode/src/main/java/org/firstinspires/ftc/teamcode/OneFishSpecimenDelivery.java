package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OneFishSpecimenDelivery {

    private Servo pivot = null;
    private Servo claw = null;


    private LinearOpMode linearOpMode;

    public final double INTAKE_PITCH = 0.1;
    public final double DELIVERY_PITCH = 0.0;

    public final double CLAW_CLOSE = 0.0;
    public final double CLAW_OPEN = 0.5;

    private double targetPosition;
    private double currentPosition;

    public final double RETRACT_TIMEOUT = 7;

    public boolean isDisabled = false;

    private double initAttempts = 0;

    private ElapsedTime time = new ElapsedTime();



    public OneFishSpecimenDelivery(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){
        try {
            pivot  = linearOpMode.hardwareMap.get(Servo.class, "specimenPivot");
            claw = linearOpMode.hardwareMap.get(Servo.class, "specimenClaw");
//            claw.scaleRange(0.0, 0.25);


        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public double getClawPosition() {return claw.getPosition();}

    public void setClawPosition(double p){ // 0-1
        claw.setPosition(p);
    }
    public void clawOpen(){
        setClawPosition(0.5);
    }
    public void clawClose(){
        setClawPosition(1);
    }

    public double getPivotPosition() {return pivot.getPosition();}

    public void setPivotPosition(double p){ // 0-1
        pivot.setPosition(p);
    }
    public void pitchToIntake(){
        setPivotPosition(INTAKE_PITCH);
    }
    public void pitchToDelivery(){
        setPivotPosition(DELIVERY_PITCH);
    }

    public void setPivotTarget(double p, double timeElapsed){ // 0-1
//        pivot.setPosition();
        //-2*Math.pow(timeElapsed, 3)+3 * Math.pow(timeElapsed, 2)    GOES FROM 0 to 1 based on time in a cubic
    }

}
