package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.IntakeFunctions;
import org.firstinspires.ftc.teamcode.OneFishSampleDelivery;

@TeleOp(name="OneFish Sample Delivery Testing", group="TEST")
public class OneFishSampleDeliveryTesting extends LinearOpMode {
    private OneFishSampleDelivery delivery;
    private boolean isHeightTesting = true;
    private boolean isManualHeightControl = false;
    private boolean isManualPitchControl = false;

    private int targetHeight = 0;
    private String pitchPositionName = "N/A";
    private int pitchPositionIndex = 0;

    private ElapsedTime inputTimer = new ElapsedTime();



    public void runOpMode() {
        delivery = new OneFishSampleDelivery(this, true);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addLine("Gamepad 1 (a) -> Height Testing");
            telemetry.addLine("Gamepad 1 (b) -> Height Manual Control");
            telemetry.addLine("Gamepad 1 (x) -> Pitch & Claw Manual Control");
            telemetry.addLine();
            telemetry.addLine();

            if(gamepad1.a){
                isHeightTesting = true;
                isManualHeightControl = false;
                isManualPitchControl = false;
            }
            if(gamepad1.b){
                isHeightTesting = false;
                isManualHeightControl = true;
                isManualPitchControl = false;
            }
            if(gamepad1.x){
                isHeightTesting = false;
                isManualHeightControl = false;
                isManualPitchControl = true;
            }

            if(isHeightTesting) {
                telemetry.addLine("   Delivery Slides Are Floating.");
                telemetry.addLine();
                telemetry.addData("      Height in ticks:", delivery.getMotorPosition());
            }

            if(isManualHeightControl) {
                telemetry.addLine("   Use Gamepad 1 left stick to control target height");
                telemetry.addLine();
                telemetry.addData("      Height in ticks:", delivery.getMotorPosition());
                targetHeight += (int)(gamepad1.left_stick_y * 20);
                targetHeight = -Math.min(2000, Math.max(0, Math.abs(targetHeight)));
                delivery.setSlidesTargetPosition(targetHeight);
                delivery.PControlPower(3);
            }

            if(isManualPitchControl) {
                telemetry.addLine("   Gamepad 1 (RIGHT_TRIGGER) -> Close Claw");
                telemetry.addLine("   Gamepad 1 (LEFT_TRIGGER) -> Open Claw");
                telemetry.addLine("   Gamepad 1 (DPAD_UP) -> Increment through list of pitch positions sequentially");
                telemetry.addLine("   Gamepad 1 (DPAD_DOWN) -> Decrement through list of pitch positions sequentially");
                telemetry.addLine();
                telemetry.addLine();
                telemetry.addData("      Height in ticks:", delivery.getMotorPosition());
                telemetry.addLine();
                telemetry.addLine("      Pitch Position: " + pitchPositionName);

                if(gamepad1.right_trigger > 0.1){
                    delivery.clawClose();
                }
                if(gamepad1.right_trigger > 0.1){
                    delivery.clawOpen();
                }
                if(gamepad1.dpad_up && inputTimer.milliseconds() > 500){
                    inputTimer.reset();
                    pitchPositionIndex++;
                }
                if(gamepad1.dpad_down && inputTimer.milliseconds() > 500){
                    inputTimer.reset();
                    pitchPositionIndex--;
                }
                pitchPositionIndex = Math.min(2, Math.max(0, pitchPositionIndex));

                if(pitchPositionIndex == 0){
                    delivery.pitchToSpecimenIntake();
                    pitchPositionName = "INTAKE";
                }
                if(pitchPositionIndex == 1){
                    delivery.pitchToSpecimenDeliver();
                    pitchPositionName = "DELIVER";
                }
                if(pitchPositionIndex == 2){
                    delivery.pitchToVertical();
                    pitchPositionName = "VERTICAL";
                }

                delivery.setSlidesTargetPosition(targetHeight);
                delivery.PControlPower(3);
            }

            telemetry.update();
        }
    }
}
