package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
public class Intake {
    private DcMotor pitchMotor = null;
    private Servo extensionServo = null;
    private Servo intakeServo = null;


    private LinearOpMode linearOpMode;

    private final double CLICKS_PER_DEGREE = 2.0883333333333;
    private final double CLICKS_PER_METER = 2492.788;
    private final int MM_PER_METER = 1000;

    private final double MIN_EXTENSION = 0.0;
    private final double MAX_EXTENSION = 1.0;

    private final double PITCH_POWER_MULTIPIER = 0.8;

    private final int TICK_LOW_POWER_DISTANCE = 200;

    private double targetLengthCM;
    private double targetAngleDegrees;
    private int targetAngle;
    private double targetLength;
    private double targetPositionX;
    private double targetPositionY;
    private double currentPosition;

    public final double TICK_STOP_THRESHOLD = 20;
    public final int BOTTOM_POSITION = (int)(2 / MM_PER_METER * CLICKS_PER_METER); //2mm

    private ElapsedTime time = new ElapsedTime();



    public Intake(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){//4mm
        try {
            pitchMotor = linearOpMode.hardwareMap.get(DcMotor.class, "intakePitch");
            extensionServo = linearOpMode.hardwareMap.get(Servo.class, "intakeExtension");
            intakeServo = linearOpMode.hardwareMap.get(Servo.class, "intakeEndEffector");
            extensionServo.scaleRange(0.0, 0.25);
            pitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }catch(NullPointerException e){
            linearOpMode.telemetry.addLine("Couldn't find intake");
        }
    }

    /**
     * set (x, y) then use updateArmPosition();
     * @param x relative x position from pole
     * @param y relative y position from pole
     */
    public void setCartesianTarget(double x, double y){
        targetPositionX = x;
        targetPositionY = y;

        updatePolarTarget();
    }

    private void updatePolarTarget(){
        setTargetAngle(Math.atan(targetPositionY/targetPositionX));
        setTargetLength(Math.sqrt(Math.pow(targetPositionX, 2) + Math.pow(targetPositionY, 2))); // sqrt( x^2 + y^2  )
    }

    private void setTargetAngle(double theta){
        targetAngleDegrees = theta;
        targetAngle = (int)(targetAngleDegrees * CLICKS_PER_DEGREE);
    }

    private void setTargetLength(double cm){
        targetLengthCM = cm;
        targetLength = CMToServoExtenderPosition(targetLengthCM);
        targetLength = Math.max(MIN_EXTENSION, Math.min(MAX_EXTENSION, targetLength));
    }

    private void updateAngle(){
        PControlPower();
    }

    private void updateLength(){
        extensionServo.setPosition(targetLength);
    }

    public void updateArmPosition(){
        updateAngle();
        updateLength();
    }

    public int getPitchMotorPosition(){ return pitchMotor.getCurrentPosition(); }

    public double getTargetAngle(){
        return targetAngleDegrees;
    }

    private static Double CMToServoExtenderPosition(double extension) {
        double a = -78.26;
        double b = 43.63;
        double c = 58.52 - extension;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            // No real solutions
            return 0.0;
        } else if (discriminant == 0) {
            // One solution
            double x = -b / (2 * a);
            return x >= 0 ? x : null; // Return if it's positive
        } else {
            // Two solutions
            double sqrtDiscriminant = Math.sqrt(discriminant);
            double x1 = (-b - sqrtDiscriminant) / (2 * a);
            return x1;
        }
    }
    public void PControlPower(){
        double error = targetAngle - pitchMotor.getCurrentPosition();
        double power = (Math.abs(error) / TICK_LOW_POWER_DISTANCE);

        power = Math.max(0, Math.min(1, power));
        if(error < 0) power *= -1;

        pitchMotor.setPower(power * PITCH_POWER_MULTIPIER);
        linearOpMode.telemetry.addData("Power: ",power);
        linearOpMode.telemetry.addData("Error: ",error);
        linearOpMode.telemetry.update();
    }

}
