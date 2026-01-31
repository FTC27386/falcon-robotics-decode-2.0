package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static final double HOOD_MIN_POSITION = 0;
    public static final double HOOD_MAX_POSITION = 0.57633333333;
    public static final double FLYWHEEL_MIN_SPEED = 0;
    public static final double FLYWHEEL_MAX_SPEED = 2200;
    public static double shooter_tolerance = 30;
    public static double shooter_kP = -0.004;
    public static double shooter_kD = 0.00000000;
    public static double shooter_kV = -0.00039;
    public static double shooter_kS = 0.056; // "lower limit" power
    public static double TURRET_MAX_POW = 1;
    public static double TURRET_TOLERANCE_RADIANS = 0.05;
    public static double TURRET_CONVERSION_FACTOR_RADIANS;
    public static double turret_kP = 0.02; // proportional gain (main steering force)
    public static double turret_kD = 0.0005; // derivative gain (motion damper)
    public static double offset_between_servos = 0;
    public static double rpm_conversion_factor = (double)(Math.PI * 2) * (double)(1/60);
}