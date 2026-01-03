package org.firstinspires.ftc.teamcode.Utility;

public class slewRateLimiter {

    private double max_rate;
    double input;
    double previousvalue;

    public slewRateLimiter(double max_rate)
    {
        this.max_rate = max_rate;
        previousvalue = 0;
    }

    public double calculate(double input)
    {
        previousvalue = Math.min((previousvalue+max_rate),(input));
        return previousvalue;
    }
}
