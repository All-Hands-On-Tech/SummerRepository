package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.DrivetrainFunctions;


//TeleOp annotation labels the opmode, and sorts it aphabetically based on group
@TeleOp(name = "SIMPLE Teleop", group = "Summer")

//class header extends LinearOpMode
public class SimpleTeleop extends LinearOpMode {

    //Motor object variable declaration
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;

    private DrivetrainFunctions drivetrainFunctions;

    //runOpMode is executed after driver presses INIT
    @Override
    public void runOpMode() {
        //The following code executes ONCE when the driver presses INIT

        drivetrainFunctions = new DrivetrainFunctions(this);

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

        while(opModeIsActive()) {
            //control motors
            drivetrainFunctions.Move(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
        }
    }
}
