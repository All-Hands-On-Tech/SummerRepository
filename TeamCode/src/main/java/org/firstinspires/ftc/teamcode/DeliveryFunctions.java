package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class DeliveryFunctions {
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private LinearOpMode linearOpMode;
    private double CLICKS_PER_METER = 2492.788;

    private double targetPosition;
    private double error;
    private double currentPosition;




    public DeliveryFunctions(LinearOpMode l)
    {
        linearOpMode = l;
    }

    private void Initialize(){
        leftSlide  = linearOpMode.hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide  = linearOpMode.hardwareMap.get(DcMotor.class, "rightSlide");
    }

//    private void SlidesTo(double meters){
//        while(error <)
//        error = targetPosition - currentPosition;
//    }
}
