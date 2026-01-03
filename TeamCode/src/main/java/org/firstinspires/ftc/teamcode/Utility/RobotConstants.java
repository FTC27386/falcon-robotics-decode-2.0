
package org.firstinspires.ftc.teamcode.Utility;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class RobotConstants {

        public enum ALLIANCE_COLOR
        {
                BLUE, RED
        }

        public static ALLIANCE_COLOR current_color = ALLIANCE_COLOR.RED;

        public static void setAutoEndPose(Pose endPose)
        {
                autoEndPose = endPose;
        }
        public static void setCurrent_color(ALLIANCE_COLOR color)
        {
                current_color = color;
        }

        public static String first_shooter_motor_name = "flywheel_top",
                second_shooter_motor_name = "flywheel_bottom",
                left_turret_servo_name = "left_turret_servo",
                right_turret_servo_name = "right_turret_servo",
                hood_servo_name = "hood",
                intake_motor_name = "intake",
    intake_servo_name = "intake_pivot_servo",
                left_front_drive_motor_name = "front_left_drive",
                right_front_drive_motor_name = "front_right_drive",
                left_back_drive_motor_name = "back_left_drive",
                right_back_drive_motor_name = "back_right_drive",
                transfer_servo_name = "blocker",
                camera = "Webcam 1",
                turret_encoder_name = "turret_encoder",
    lift_motor_name = "lift_motor",
    lift_servo_name = "latch_servo",
    right_lift_servo_name = "right_lift_servo";

        public static double
                servo_offset = 0,
                shooterTolerance = 30,
                vera_coefficient = .4167,
                turret_offset_inch = -0.63696,

                lift_kP = 0.001, //0.0003
                lift_kD = 0.0,
                lift_kF = 0.3,
                shooter_kP = -0.002,
                shooter_kD = 0.00000000,
                shooter_kV = -0.00039,
                shooter_kS = 0.056, //"lower limit" power
                turret_kP = 0,
                turret_kD = 0.0,
                turret_kL = 0,
                offset_between_servos = 0,
                turret_conversion_factor_DEGREES = (0.0015962441314554),
                turret_conversion_factor_RADIANS = (double)(1/5) * (double)(170/60) * (double)(1/355) * (double)(360/(2*Math.PI)), // for mason's weird ahh
                rpm_conversion_factor = (double)(Math.PI * 2) * (double)(1/60),
                far_flywheel_speed = -310,
                close_flywheel_speed = -250,
                transfer_closed_pos = 0.5,
                transfer_open_pos = .25,
                latch_close_pos = 0,
                latch_open_pos = 1,
                pivot_up_pos = 0,
                pivot_down_pos = 1;
        public static Pose autoEndPose;
        public enum robotState
        {
                IDLE,
                INTAKING,
                SHOOTING,
                TRANSFER,
                AIMING,
                ENDGAME,
        }
        public static int top_climb_position = 13000; // 13400 is the real maximum




}
