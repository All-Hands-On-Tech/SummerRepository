package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
public class TouchpadFunctions {
    public static final double TOUCHPADXMULTIPLIER = 0.5;
    public static final double TOUCHPADYMULTIPLIER = 0.1;
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
        gamepadNum = gamepadNumber;
        linearOpMode = l;
        Initialize();

    }


    private void Initialize(){
        currentGamepad = new Gamepad();
        prevGamepad = new Gamepad();
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

    public void CollectPreviousInput(){
        prevGamepad.copy(currentGamepad);
    }

    public void CollectCurrentInput(){
        currentGamepad.copy(gamepad);
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
