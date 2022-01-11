package org.firstinspires.ftc.teamcode.util;

public class Utility {
    //if the stick value is between -.05 and .05, then call it 0
    public static double deadStick(double value) {
        if (value > -0.05 && value < 0.05) {
            value = 0;
        }
        return value;
    }

    //if degrees is > 180 or< -180 then add or subtract 360 to bring it into that range
    public static double wrapDegrees360(double degrees) {
        if (degrees > 180) degrees = degrees - 360;

        if (degrees < -180) degrees = degrees + 360;

        return degrees;
    }

    public static double clipToRange(double entryNumber, double max, double min) {
        if (entryNumber > max) {
            entryNumber = max;
        } else if (entryNumber < min) {
            entryNumber = min;
        }
        return entryNumber;
    }
}