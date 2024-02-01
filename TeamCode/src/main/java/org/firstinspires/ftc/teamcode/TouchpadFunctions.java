package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
public class TouchpadFunctions {
    public static final double TOUCHPADXMULTIPLIER = 0.05;
    public static final double TOUCHPADYMULTIPLIER = 0.01;
    private Gamepad prevGamepad;
    private Gamepad currentGamepad;

    private LinearOpMode linearOpMode;
    private Gamepad gamepad;
    private int gamepadNum;
    private float onPressX;
    private float onPressY;

    private float XSwipe;
    private float YSwipe;


    public TouchpadFunctions(LinearOpMode l, int gamepadNumber)
    {
        linearOpMode = l;
        Initialize();
        gamepadNum = gamepadNumber;

    }


    private void Initialize(){
        if(gamepadNum == 1){
            gamepad = linearOpMode.gamepad1;
        } else {
            gamepad = linearOpMode.gamepad2;
        }
    }

    private void Reinitialize(){
        if(gamepadNum == 1){
            gamepad = linearOpMode.gamepad1;
        } else {
            gamepad = linearOpMode.gamepad2;
        }
    }

    public boolean getTouchpadDown(){
         gamepad.copy(currentGamepad);

         if(prevGamepad != null && !prevGamepad.touchpad_finger_1 && currentGamepad.touchpad_finger_1){
             //DOWN
             onPressX = currentGamepad.touchpad_finger_1_x;
             onPressY = currentGamepad.touchpad_finger_1_y;
             return true;
         } else{
             return false;
         }
    }

    public boolean getTouchpadUp(){
        gamepad.copy(currentGamepad);
        if(prevGamepad != null && prevGamepad.touchpad_finger_1 && !currentGamepad.touchpad_finger_1){
            //UP
            return true;
        } else{
            return false;
        }
    }

    public boolean getTouchpad(){
        return gamepad.touchpad_finger_1;
    }

    public void updateTouchpadStateAtEndOfLoop(){
        gamepad.copy(prevGamepad);
    }

    public float getXSwipe(){
        XSwipe = currentGamepad.touchpad_finger_1_x - onPressX;
        return XSwipe;
    }

    public float getYSwipe(){
        YSwipe = currentGamepad.touchpad_finger_1_y - onPressY;
        return YSwipe;
    }

}
