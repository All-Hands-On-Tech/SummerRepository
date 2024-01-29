package org.firstinspires.ftc.teamcode.Autonomous.Other;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpmode;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.firstinspires.ftc.teamcode.VisionFunctions;

@Autonomous(name="Auto apriltag test", group="ARed")
public class AUTOAPRILTAGTEST extends LinearOpMode {

    //logan was here

    VisionFunctions apriltagFunctions;

    DrivetrainFunctions drivetrainFunctions;

    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        apriltagFunctions=new VisionFunctions(this);
        drivetrainFunctions = new DrivetrainFunctions(this);
        waitForStart();
        time.startTime();
        if (isStopRequested()) return;

        sleep(1000);
        apriltagFunctions.startDetectingApriltags();

        MoveToTagForSeconds(apriltagFunctions.BLUE_1_TAG, 1000, 10);


    }

    private void MoveToTagForSeconds(int desiredTag, double seconds, float distanceinInches){
        time.reset();
        while(time.seconds() < seconds){
            apriltagFunctions.DetectAprilTag(desiredTag);
            if(apriltagFunctions.detectedTag != null){
                float moveProfile[] = apriltagFunctions.moveToTag(desiredTag, distanceinInches);
                drivetrainFunctions.Move(moveProfile[0], moveProfile[1], moveProfile[2], 0.5);
                telemetry.addLine("Moving");
            } else {
                telemetry.addLine("No detection");
            }
            telemetry.update();
        }
    }

}