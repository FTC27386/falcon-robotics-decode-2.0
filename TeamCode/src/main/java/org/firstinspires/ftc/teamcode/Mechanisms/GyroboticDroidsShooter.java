package org.firstinspires.ftc.teamcode.Mechanisms;

/*
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.PASS_THROUGH_POINT_RADIUS;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.SCORE_ANGLE;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.SCORE_HEIGHT;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.gravity;

import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.Utility.UtilMethods;
 */

public class GyroboticDroidsShooter {
    /*
    double distance = realTurretPose.distanceFrom(goalPose); // distance from release point

    double dx = goalPose.getX() - realTurretPose.getX(); // horizontal distance from release point in field coordinates
    double dy = goalPose.getY() - realTurretPose.getY(); // vertical distance from release point in field coordinates

    double field_angle = Math.atan2(dy, dx); // angle between release point and goal
    Vector robotToGoalVector = new Vector(distance, field_angle); // vector from release point to goal

    // effective horizontal distance from release point
    double x_point = distance - PASS_THROUGH_POINT_RADIUS; // stationary pass-through horizontal distance (in)
    if (x_point <= 0) return;

    // vertical displacement (can be positive or negative)
    double y_point = SCORE_HEIGHT; // pass-through height (in)
    double a = SCORE_ANGLE; // radians

    // launch angle
    double hoodAngle = Math.atan(2.0 * y_point / x_point - Math.tan(a));

    double den1 =
            2.0 * Math.pow(Math.cos(hoodAngle), 2)
                    * (x_point * Math.tan(hoodAngle) - y_point);

    if (den1 <= 0) return;

    double flywheelSpeed = Math.sqrt(gravity * x_point * x_point / den1);

    Vector robotVelocity = follower.getVelocity();

    double coordinateTheta =
            robotVelocity.getTheta() - robotToGoalVector.getTheta();

    double vrr =
            -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
    // robot radial velocity (toward/away from goal) (in/s)

    double vrt =
            Math.sin(coordinateTheta) * robotVelocity.getMagnitude();
    // robot tangential velocity (sideways) (in/s)

    // ---------- velocity compensation ----------
    double vx = flywheelSpeed * Math.cos(hoodAngle); // horizontal component
    double vz = flywheelSpeed * Math.sin(hoodAngle); // vertical component
    if (vx < 0) return;

    double time = x_point / vx;                      // time to reach x using stationary solution

    double ivr = vx + vrr; // Add robot radial velocity (field-plane forward direction) to horizontal component

    double nvr = Math.hypot(ivr, vrt); // Combine forward and sideways robot motion to get total horizontal speed
    double ndr = nvr * time;           // Convert total horizontal speed into an effective horizontal distance
    // as though the projectile problem only cares about the length of the horizontal path

    // New launch angle using total horizontal speed and old vertical component
    // Holding vertical velocity constant naturally damps hood adjustment
    hoodAngle = Math.atan2(vz, nvr);

    double den2 = // Denominator for compensated launch speed
            2.0 * Math.pow(Math.cos(hoodAngle), 2)
                    * (ndr * Math.tan(hoodAngle) - y_point);

    if (den2 <= 0) return;
    // New (compensated) launch speed
    flywheelSpeed =
            Math.sqrt(gravity * ndr * ndr / den2);

    // Turret lead
    double turretVelCompOffset = Math.toDegrees(Math.atan2(vrt, ivr));
    double turretAngle = -UtilMethods.AngleDifference(Math.toDegrees(unnormalizedHeading), 0)
            + (90 - Math.toDegrees(field_angle))
            + turretVelCompOffset;

    setpoint = new DrivetrainSystem.ShooterSetpoint(flywheelSpeed, hoodAngle, turretAngle);

     */
}
