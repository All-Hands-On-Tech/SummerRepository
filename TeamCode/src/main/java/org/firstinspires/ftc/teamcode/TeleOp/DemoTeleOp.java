package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


//TeleOp annotation labels the opmode, and sorts it aphabetically based on group
@TeleOp(name = "DEMO Teleop", group = "DEMO")

//class header extends LinearOpMode
public class DemoTeleOp extends LinearOpMode {

    //Motor object variable declaration
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;

    //runOpMode is executed after driver presses INIT
    @Override
    public void runOpMode (){
        //The following code executes ONCE when the driver presses INIT

        //Get motor objects from config file
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");

        //Configure motors (OPTIONAL)
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Wait for driver to press START BUTTON
        waitForStart();

        //control motors
        if(-gamepad1.left_stick_y > 0.05){
            rightFrontMotor.setPower(-gamepad1.left_stick_y);
            rightBackMotor.setPower(-gamepad1.left_stick_y);
            leftFrontMotor.setPower(-gamepad1.left_stick_y);
            leftBackMotor.setPower(-gamepad1.left_stick_y);
        } else if (gamepad1.left_stick_x > 0.05) {
            rightFrontMotor.setPower(-gamepad1.left_stick_x);
            rightBackMotor.setPower(gamepad1.left_stick_x);
            leftFrontMotor.setPower(gamepad1.left_stick_x);
            leftBackMotor.setPower(-gamepad1.left_stick_x);
        } else if (gamepad1.right_stick_x > 0.05){
            rightFrontMotor.setPower(-gamepad1.left_stick_x);
            rightBackMotor.setPower(-gamepad1.left_stick_x);
            leftFrontMotor.setPower(gamepad1.left_stick_x);
            leftBackMotor.setPower(gamepad1.left_stick_x);
        } else{
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
        }
    }
}
