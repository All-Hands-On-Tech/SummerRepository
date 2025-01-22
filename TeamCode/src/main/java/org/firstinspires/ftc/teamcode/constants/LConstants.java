package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.002;
        ThreeWheelConstants.strafeTicksToInches = 0.002;
        ThreeWheelConstants.turnTicksToInches = 0.002;
        ThreeWheelConstants.leftY = 7.75;
        ThreeWheelConstants.rightY = -7.75;
        ThreeWheelConstants.strafeX = -5.5;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "LBLE";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "RBRE";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "RFBE";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




