package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static final double HOOD_MIN_POSITION = 0;
    public static final double HOOD_MAX_POSITION = 0.57633333333;
    public static double SHOOTER_TOLERANCE = 30;
    public static double shooter_kP = -0.004;
    public static double shooter_kD = 0.00000000;
    public static double shooter_kV = -0.00039;
    public static double shooter_kS = 0.056; // "lower limit" power
    public static double TURRET_MAX_POW = 1;
    public static double TURRET_TOLERANCE_RADIANS = 0.05;
    public static double TURRET_MIN = 0;
    public static double TURRET_MAX = 0;
    //public static double TURRET_CONVERSION_FACTOR_RADIANS = 0.0057674767602;
    public static double TURRET_CONVERSION_FACTOR_RADIANS = 0.000203026868985;
    public static double turret_kP = -2.8; // proportional gain (main steering force) used to be -4.5
    public static double turret_kD = 0.0; // derivative gain (motion damper) used to be 0.05
    public static double turret_kS = 0;

    public static double turretMinEndpoint = Math.PI;
    public static double turretMaxEndpoint = 3* Math.PI;
}