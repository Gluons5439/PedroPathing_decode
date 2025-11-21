package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.0005;
        TwoWheelConstants.strafeTicksToInches = 0.0005;
        TwoWheelConstants.forwardY = 6.8260;
        TwoWheelConstants.strafeX = -3.3660;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "frontLeftMotor";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "backRightMotor";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.IMU_HardwareMapName = "pinpoint";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
    }
}




