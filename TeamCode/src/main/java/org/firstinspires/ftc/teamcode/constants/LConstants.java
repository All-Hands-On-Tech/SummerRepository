package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .001989436789; /** use individual tuning programs*/
        ThreeWheelConstants.strafeTicksToInches = .001989436789;
        ThreeWheelConstants.turnTicksToInches = .001989436789;
        ThreeWheelConstants.leftY = 7.75;  /** need to be physically measured*/
        ThreeWheelConstants.rightY = -7.75;
        ThreeWheelConstants.strafeX = -5.5;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "LBLE";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "RFBE";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "RBRE";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE; /** use Localization Test*/
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




