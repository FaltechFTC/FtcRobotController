package org.firstinspires.ftc.teamcode;


public class Pose {
    public double x, y, heading;
    public Pose (double x, double y, double heading) {
        this.x=x;
        this.y=y;
        this.heading=heading;
    }
    public boolean isInTolerance(Pose pose, double toleranceClicks, double toleranceDegrees) {
        return  (Math.abs(x-pose.x)<toleranceClicks) &&
                (Math.abs(y-pose.y)<toleranceClicks) &&
                (Math.abs(heading-pose.heading)<toleranceDegrees);
    }
    public void shiftPoseXYbyDegrees(double degrees) {
        //calculate drive direction
        double direction = Math.atan2(y, x);
        //offset by 45 degrees
        direction += Math.toRadians(degrees);
        double power = Math.sqrt(x*x+y*y);
        //calculate back to strafe and forward
        y = Math.sin(direction)*power;
        x = Math.cos(direction)*power;
    }
    public Pose clone() {
        return new Pose(x,y,heading);
    }
}
