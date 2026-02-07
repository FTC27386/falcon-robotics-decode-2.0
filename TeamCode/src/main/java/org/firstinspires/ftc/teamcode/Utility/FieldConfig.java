package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class FieldConfig {
    public static final Pose TARGET_POS_RED = new Pose(144, 144);
    public static final Pose TARGET_POS_BLUE = TARGET_POS_RED.mirror();
    public static final Pose RELOC_POSE_RED = new Pose(135, 9, Math.toRadians(90));
    public static final Pose RELOC_POSE_BLUE = new Pose(9, 9, Math.toRadians(90));
    public static final double ZONE_BUFFER = 7.5 * Math.sqrt(2);
}