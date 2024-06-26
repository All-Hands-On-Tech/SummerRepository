package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class XDrivetrainFunctions extends DrivetrainFunctions{
    //constructor just runs super constructor
    public XDrivetrainFunctions(LinearOpMode l)
    {
        super(l);
    }

    //method overrides!!
    public void Move(float x, float y, float rx, double speedScalar){
        //x: forward/backward
        //y: left/right
        //rx: rotation
        if(isDisabled)
            return;
        y = -y;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFrontDrive.setPower(((-y - x + rx) / denominator) * speedScalar);
        leftBackDrive.setPower(((y - x + rx) / denominator) * speedScalar);
        rightFrontDrive.setPower(((-y + x + rx) / denominator) * speedScalar);
        rightBackDrive.setPower(((y + x + rx) / denominator) * speedScalar);
    }

    public void MoveFieldOriented (float x, float y, float rx, double speedScalar){
//        if(isDisabled)
//            return;
//
//        y = -y;
//
//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        double rotY = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);
//        double rotX = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
//
//        rotX = rotX * 1.1;
//
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        leftFrontDrive.setPower((rotX + rotY + rx) / denominator * speedScalar);
//        leftBackDrive.setPower((rotX - rotY + rx) / denominator * speedScalar);
//        rightFrontDrive.setPower((rotX - rotY - rx) / denominator * speedScalar);
//        rightBackDrive.setPower((rotX + rotY - rx) / denominator * speedScalar);

        getLinearOpMode().telemetry.addLine("XDrive Field Oriented Drive is WIP");
    }


}
