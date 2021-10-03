package org.firstinspires.ftc.teamcode;

public class Utility {
    //if the stick value is between -.05 and .05, then call it 0
    public double deadStick(double value) {
        if (value > -0.05 && value < 0.05){
            value=0;
        }
        return value;
    }
    //if degrees is > 180 or< -180 then add or subtract 360 to bring it into that range
    public double wrapDegrees360(double degrees) {
        if (degrees > 180) degrees = degrees-360;

        if (degrees < -180) degrees = degrees+360;

        return degrees;
    }
}