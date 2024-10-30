package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
public class Intake {
    private DcMotor pitchMotor = null;
    private Servo extensionServo = null;
    private Servo intakeServo = null;

    private final int PITCH_ZERO = 150; // angle in degrees from coordinate zero

    private LinearOpMode linearOpMode;

    public final double CLICKS_PER_DEGREE = 3.5;
    private final double CLICKS_PER_CM = 24.92788;
    private final int MM_PER_METER = 1000;

    private final double MIN_EXTENSION = 0.01;
    private final double MAX_EXTENSION = 0.3;

    private final int MIN_ANGLE = -1700;
    private final int MAX_ANGLE = 0;

    private final double PITCH_POWER_MULTIPIER = 0.8;

    private final int TICK_LOW_POWER_DISTANCE = 200;

    private double targetLengthCM;
    private double targetAngleDegrees;
    private int targetAngle;
    private double targetLength;
    private double targetPositionX;
    private double targetPositionY;
    private double postProcessTargetX;
    private double currentPosition;

    private double robotDeltaX = 0.0;
    private double globalTargetOffsetX = 0.0;

    public final double TICK_STOP_THRESHOLD = 20;

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
            extensionServo.scaleRange(0.0, 1.0);
            pitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            pitchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



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

    public double getCartesianTargetXFromPolar(){
        return targetLength * Math.cos(targetAngleDegrees);
    }
    public double getCartesianTargetYFromPolar(){
        return targetLength * Math.sin(targetAngleDegrees);
    }

    private void updatePolarTarget(){
        setTargetAngle(Math.toDegrees(XYToTheta(targetPositionY, postProcessTargetX)));
        setTargetLength(Math.sqrt(Math.pow(postProcessTargetX, 2) + Math.pow(targetPositionY, 2))); // sqrt( x^2 + y^2  )
        linearOpMode.telemetry.addData("Target length: ",targetLength);
        linearOpMode.telemetry.addData("Target angle: ",targetAngleDegrees);
    }

    public void setEndEffectorSpeed(float speed){
        intakeServo.setPosition(speed);
    }

    public void setTargetAngle(double theta){
        targetAngleDegrees = theta;
        targetAngle = (int)((targetAngleDegrees - PITCH_ZERO) * CLICKS_PER_DEGREE);
        targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, targetAngle));
    }

    public void setTargetAngleTicks(int ticks){
        targetAngle = ticks;
//        targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, targetAngle));
    }

    public void setTargetLength(double cm){
        targetLengthCM = cm;
        targetLength = CMToServoExtenderPosition(targetLengthCM);
        targetLength = Math.max(MIN_EXTENSION, Math.min(MAX_EXTENSION, targetLength));
    }

    public void setTargetLengthServo(double pos){
        targetLength = pos;
        targetLength = Math.max(MIN_EXTENSION, Math.min(MAX_EXTENSION, targetLength));
    }

    public void incrementTargetAngleTicks(int ticks){
        targetAngle += ticks;
        targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, targetAngle));
    }

    public void setTargetAngleTicks(int ticks){
        targetAngle = ticks;
        targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, targetAngle));
    }

    public void updateAngle(){
        PControlPower();
        linearOpMode.telemetry.addData("Intake Pitch: ", targetAngle);
    }

    public void updateLength(){
        extensionServo.setPosition(targetLength);
        linearOpMode.telemetry.addData("Intake Length: ", targetLength);
    }

    public void updateArmPosition(boolean globalTargeting){
        if(globalTargeting){
            globalTargetOffsetX += robotDeltaX / CLICKS_PER_CM;
            postProcessTargetX += globalTargetOffsetX;
        }else{
            robotDeltaX = 0.0;
            postProcessTargetX = targetPositionX; //FIXME: Determine whether or not to keep this line
            /*

            If this line is removed the target position will not return to the inputted relative position, it will stay wherever the global targeting process has left it.

            */
        }
        updateAngle();
        updateLength();
    }

    public void resetGlobalOffset(){
        postProcessTargetX = targetPositionX;
    }

    public void inputDeltaX(double deltaX){
        this.robotDeltaX = deltaX;
    }

    public int getPitchMotorPosition(){ return pitchMotor.getCurrentPosition(); }

    public double getTargetAngle(){
        return targetAngleDegrees;
    }

    private static Double XYToTheta(double y, double x){    //Domain: all reals Range: -pi/2 to 3pi/2
        if(x>=0) {
            return Math.atan(y / x);
        }
        else {
            return Math.atan(y / x) + Math.PI;
        }
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
//        linearOpMode.telemetry.update();
    }

}
