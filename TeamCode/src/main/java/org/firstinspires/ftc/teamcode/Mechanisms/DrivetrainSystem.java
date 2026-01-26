package org.firstinspires.ftc.teamcode.Mechanisms;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.Utility.FieldConfig.gravity;
import static org.firstinspires.ftc.teamcode.Utility.RobotConfig.zone_buffer;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.PASS_THROUGH_POINT_RADIUS;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.SCORE_ANGLE;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.SCORE_HEIGHT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;
import org.firstinspires.ftc.teamcode.Utility.UtilMethods;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

public class DrivetrainSystem extends SubsystemBase {
    public Pose
            currentPose = new Pose(0, 0, Math.toRadians(90)),
            calculatedPose,
            realTurretPose = new Pose(0,0),
            goalPose,
            targetPose;
    public Follower follower;
    private final InterpLUT LUT1 = new InterpLUT();
    private final InterpLUT LUT2 = new InterpLUT();
    private final InterpLUT LUT3 = new InterpLUT();
    public boolean shooterOff = false;

    public double
            x,
            y,
            distance,
            distanceX,
            distanceY,
            dist,
            heading,
            unnormalizedHeading,
            field_angle,
            hoodAngle,
            turretVelCompOffset,
            flywheelSpeed,
            other_distance;
    public Supplier<Pose> poseSupplier = this::getCurrentPose;
    boolean robotCentricDrive = false;

    public DrivetrainSystem(HardwareMap hMap) {
        follower = Constants.createFollower(hMap);
        follower.setStartingPose(RobotConstants.autoEndPose == null ? new Pose(8, 8, Math.toRadians(90)) : RobotConstants.autoEndPose);
        follower.update();
        if (RobotConstants.current_color == null || RobotConstants.current_color == RobotConstants.ALLIANCE_COLOR.BLUE) {
            targetPose = new Pose(0, 144);
            goalPose = GOAL_POS_BLUE;
        } else {
            targetPose = new Pose(144, 144);
            goalPose = GOAL_POS_RED;
        }

        //-1400
        LUT1.add(45.00, 0.2);
        LUT1.add(55.00, 0.25);
        LUT1.add(65.00, 0.4);
        LUT1.createLUT();

        //-1500
        LUT2.add(65.00, 0.25);
        LUT2.add(75.00, 0.35);
        LUT2.add(85.00, 0.4);
        LUT2.createLUT();

        //-1600
        LUT3.add(85.00, 0.3);
        LUT3.add(90.00, 0.25);
        LUT3.add(95.70, 0.3);
        LUT3.createLUT();
    }

    @Override
    public void periodic() {
        follower.update();
        currentPose = follower.getPose();
        calculatedPose = getPredictedPose(num);
        realTurretPose = computeOffset(calculatedPose, RobotConstants.turret_offset_inches);

        x = currentPose.getX();
        y = currentPose.getY();

        distanceX = targetPose.getX() - realTurretPose.getX();
        distanceY = targetPose.getY() - realTurretPose.getY();

        heading = currentPose.getHeading();
        unnormalizedHeading = follower.getTotalHeading();
        calculateShotVectorAndUpdateTurret();
    }
    public void calculateShotVectorAndUpdateTurret() {

        final double g = 32.174 * 12;     // gravity (in/s^2)
        double distance = realTurretPose.distanceFrom(goalPose);

        double dx = goalPose.getX() - realTurretPose.getX();
        double dz = goalPose.getY() - realTurretPose.getY();

        double field_angle = Math.atan2(dz, dx);
        Vector robotToGoalVector = new Vector(distance, field_angle);

        // effective horizontal distance from release point
        double x_point = distance - PASS_THROUGH_POINT_RADIUS; //stationary pass-through horizontal distance (in)

        // vertical displacement (can be positive or negative)
        double y_point = SCORE_HEIGHT; // pass-through height (in)

        double a = SCORE_ANGLE; // radians

        hoodAngle = Math.atan(2.0 * y_point / x_point - Math.tan(a));

        double den1 =
                2.0 * Math.pow(Math.cos(hoodAngle), 2)
                        * (x_point * Math.tan(hoodAngle) - y_point);

        if (den1 <= 0) return;

        flywheelSpeed = Math.sqrt(g * x_point * x_point / den1);

        // ---------- robot velocity ----------
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
        double vx = flywheelSpeed * Math.cos(hoodAngle);  // horizontal component
        double vz  = flywheelSpeed * Math.sin(hoodAngle); // vertical component
        double time = x_point / vx;                       // time to reach x using stationary solution

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
                Math.sqrt(g * ndr * ndr / den2);

        // Turret lead
        turretVelCompOffset = Math.atan2(vrt, ivr);
    }

    public double getDist() { return follower.getPose().distanceFrom(targetPose); }
    public double getAim() { //calculate adjusted turret angle in degrees
        return turretVelCompOffset;
    }

    public double getHood() {
        return hoodAngle;
    }

    public double getSpeed() {
        return flywheelSpeed;
    }

    public void toggleShooter()
    {
        shooterOff = !shooterOff;
    }

    public void teleOpDrive(double axial, double lateral, double yaw) {
        follower.setTeleOpDrive(
                -axial,
                -lateral,
                -yaw,
                true);
    }
    public Pose computeOffset(Pose pose, double offset)
    {
        double heading = pose.getHeading();
        double NewX = pose.getX() + Math.cos(heading) * offset;
        double NewY = pose.getY() + Math.sin(heading) * offset;
        return new Pose(NewX, NewY);
    }
    public Pose getPredictedPose(double secondsInFuture) {
        // 1. Get current state
        Pose currentPose = follower.getPose();
        Vector currentVel = follower.getVelocity(); // Field-centric velocity

        // 2. Calculate displacement (Velocity * Time)
        double deltaX = currentVel.getXComponent() * secondsInFuture;
        double deltaY = currentVel.getYComponent() * secondsInFuture;

        // 3. Return the predicted position
        return new Pose(
                currentPose.getX() + deltaX,
                currentPose.getY() + deltaY,
                currentPose.getHeading() // Heading could change
        );
    }

    public void reloc(Pose reloc) {
        follower.setPose(reloc);
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    public Pose getTargetPose() {
        return targetPose;
    }

    public void relocTargetPose(Pose reloc) {
        targetPose = reloc;
    }

    public boolean inCloseZone() {
        //return (y > Math.abs(x - 72) + 72 - zone_buffer);
        return true;
    }
    public boolean inFarZone() {
        //return (y < -Math.abs(x - 72) + 24 + zone_buffer);
        return true;
    }
}
