package org.firstinspires.ftc.teamcode.Mechanisms;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.Utility.RobotConstants.zone_buffer;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConstants.GOAL_POS_BLUE;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConstants.GOAL_POS_RED;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConstants.HOOD_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConstants.HOOD_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConstants.PASS_THROUGH_POINT_RADIUS;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConstants.SCORE_ANGLE;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConstants.SCORE_HEIGHT;
import static org.firstinspires.ftc.teamcode.opMode.teleOp.flywheel_speed;
import static org.firstinspires.ftc.teamcode.opMode.teleOp.hood_angle;
import static org.firstinspires.ftc.teamcode.opMode.teleOp.num;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;
import org.firstinspires.ftc.teamcode.Utility.ShooterConstants;
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

        // ---------- constants ----------
        final double g = 32.174 * 12;     // in/s^2
        final double EPS = 1e-6;

        // ---------- geometry ----------
        double distance = realTurretPose.distanceFrom(goalPose);

        double dx = goalPose.getX() - realTurretPose.getX();
        double dy = goalPose.getY() - realTurretPose.getY();

        double field_angle = Math.atan2(dy, dx);
        Vector robotToGoalVector = new Vector(distance, field_angle);

        // effective horizontal distance from release point
        double x_point =
                robotToGoalVector.getMagnitude() - PASS_THROUGH_POINT_RADIUS;
        x_point = Math.max(x_point, EPS);   // NEVER clamp to 0

        // vertical displacement (can be positive or negative)
        double y_point = SCORE_HEIGHT;

        double a = SCORE_ANGLE; // radians

        // ---------- initial ballistic solve ----------
        hoodAngle = clamp(
                Math.atan(2.0 * y_point / x_point - Math.tan(a)),
                HOOD_MIN_ANGLE,
                HOOD_MAX_ANGLE
        );

        double denom1 =
                2.0 * Math.pow(Math.cos(hoodAngle), 2)
                        * (x_point * Math.tan(hoodAngle) - y_point);

        if (denom1 <= EPS) {
            return; // no physical solution
        }

        flywheelSpeed =
                Math.sqrt(g * x_point * x_point / denom1);

        // ---------- robot velocity ----------
        Vector robotVelocity = follower.getVelocity();

        double coordinateTheta =
                robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent =
                -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();

        double perpendicularComponent =
                Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        // ---------- velocity compensation ----------
        double vx0 = flywheelSpeed * Math.cos(hoodAngle);
        double vz  = flywheelSpeed * Math.sin(hoodAngle);

        double time = x_point / Math.max(vx0, EPS);

        double ivr = vx0 + parallelComponent;

        double nvr = Math.max(
                Math.hypot(ivr, perpendicularComponent),
                EPS
        );

        double ndr = nvr * time;

        // ---------- recompute launch components ----------
        hoodAngle = clamp(
                Math.atan2(vz, nvr),
                HOOD_MIN_ANGLE,
                HOOD_MAX_ANGLE
        );

        double denom2 =
                2.0 * Math.pow(Math.cos(hoodAngle), 2)
                        * (ndr * Math.tan(hoodAngle) - y_point);

        if (denom2 <= EPS) {
            return;
        }

        flywheelSpeed =
                Math.sqrt(g * ndr * ndr / denom2);

        // ---------- turret lead ----------
        turretVelCompOffset = Math.atan2(perpendicularComponent, ivr);
    }

    public double getDist() { return follower.getPose().distanceFrom(targetPose); }
    public double getAim()  //calculate adjusted turret angle in degrees
    {
        field_angle = (90 - Math.toDegrees(Math.atan2(distanceY, distanceX)));
        // return Math.toDegrees((heading)-Math.PI/2) + field_angle;
        return -UtilMethods.AngleDifference(Math.toDegrees(unnormalizedHeading), 0) + field_angle;
        //equivalent: Math.toDegrees(currentpose.getHeading() - Math.PI/2)
        //(90 - Math.toDegrees(Math.atan2(distanceY,distanceX)))
    }

    public double getHood() {
        /*
        dist = getDist();
        if      (dist >= 45.00 && dist <  65.00) return LUT1.get(dist);
        else if (dist >= 65.00 && dist <  85.00) return LUT2.get(dist);
        else if (dist >= 85.00 && dist <= 95.00) return LUT3.get(dist);
        else                                     return 0;

         */
        return hoodAngle;
    }

    public double getSpeed() {
        /*
        dist = getDist();
        if (shooterOff) return 0;
        if      (dist >= 45.00 && dist <  65.00) return -1400;
        else if (dist >= 65.00 && dist <  85.00) return -1500;
        else if (dist >= 85.00 && dist <= 95.00) return -1600;
        else                                     return -1600;

         */
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
