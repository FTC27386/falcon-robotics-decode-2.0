
package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class RobotConfig {

    public enum ALLIANCE_COLOR {
        BLUE, RED
    }
    public static ALLIANCE_COLOR current_color = ALLIANCE_COLOR.BLUE;
    public static void setAutoEndPose(Pose endPose)
    {
            autoEndPose = endPose;
    }
    public static void setCurrentColor(ALLIANCE_COLOR color)
    {
            current_color = color;
    }
    public static String first_shooter_motor_name = "flywheel_top";
    public static String second_shooter_motor_name = "flywheel_bottom";
    public static String left_turret_servo_name = "left_turret_servo";
    public static String right_turret_servo_name = "right_turret_servo";
    public static String hood_servo_name = "hood";
    public static String intake_motor_name = "intake";
    public static String intake_servo_name = "intake_pivot_servo";
    public static String left_front_drive_motor_name = "front_left_drive";
    public static String right_front_drive_motor_name = "front_right_drive";
    public static String left_back_drive_motor_name = "back_left_drive";
    public static String right_back_drive_motor_name = "back_right_drive";
    public static String transfer_servo_name = "blocker";
    public static String camera = "Webcam 1";
    public static String turret_encoder_name = "turret_encoder";
    public static String lift_motor_name = "lift_motor";
    public static String lift_servo_name = "latch_servo";
    public static String limelight_name = "limelight";

    public static double shooter_tolerance = 30;
    public static double vera_coefficient = .4167;
    public static double turret_offset_inches = -0.63696;
    public static double lift_kP = 0.001; //0.0003
    public static double lift_kD = 0.0;
    public static double lift_kF = 0.3;
    public static double shooter_kP = -0.004;
    public static double shooter_kD = 0.00000000;
    public static double shooter_kV = -0.00039;
    public static double shooter_kS = 0.056; // "lower limit" power
    public static double turret_kP = 0;
    public static double turret_kD = 0.0;
    public static double turret_kL = 0;
    public static double offset_between_servos = 0;
    public static double turret_conversion_factor_DEGREES = (0.0015962441314554);
    public static double turret_conversion_factor_RADIANS = (double)(1/5) * (double)(170/60) * (double)(1/355) * (double)(360/(2*Math.PI)); // For mason's weird ahh
    public static double rpm_conversion_factor = (double)(Math.PI * 2) * (double)(1/60);

    public static Pose autoEndPose;
    public static int top_climb_position = 10400; // 13400 is the real maximum
    //public static int top_climb_position = 9500; // 13400 is the real maximum
    public double Apriltag_20x = 14.64;
    public double Apriltag_20y = 144-15.4;
    public double Apriltag_24x = 144-14.64;
    public double Apriltag_24y = 128.6;
}