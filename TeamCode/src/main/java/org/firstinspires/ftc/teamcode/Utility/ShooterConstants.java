
package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
@Config
public class ShooterConstants {

    public static Pose GOAL_POS_RED = new Pose(138, 138);

    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();

    public static double SCORE_HEIGHT = 26; //inches

    public static double SCORE_ANGLE = Math.toRadians(-30); //radians

    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches
}