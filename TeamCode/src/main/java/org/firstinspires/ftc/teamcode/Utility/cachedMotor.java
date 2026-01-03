package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class cachedMotor {
    public DcMotorEx thisMotor;
    double cachingTol;
    double currPower=0;
    public cachedMotor(DcMotorEx motor, double cachingTol )
    {
        thisMotor = motor;
        this.cachingTol = cachingTol;
    }
    public void setPower(double newPower)
    {
        if (Math.abs(currPower-newPower) > cachingTol || newPower == 0)
        {
            currPower = newPower;
        }
        thisMotor.setPower(currPower);
    }
    public int getCurrentPosition()
    {
        return thisMotor.getCurrentPosition();
    }
    public void setCachingTol(double cachingTol)
    {
        this.cachingTol = cachingTol;
    }
}
