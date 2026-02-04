package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utility.FieldConfig;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.Utility.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Utility.UtilMethods;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DrivetrainSystem extends SubsystemBase {
    SlewRateLimiter axialLimiter;
    public Follower follower;
    private Pose currentPose = new Pose(0, 0, Math.toRadians(90));
    private Pose realTurretPose = new Pose(0, 0);
    private Pose targetPose;
    private double x;
    private double y;
    ShooterSetpoint setpoint = new ShooterSetpoint(0, 0, 0, false);

    public DrivetrainSystem(HardwareMap hMap) {
        follower = Constants.createFollower(hMap);
        follower.setStartingPose(RobotConfig.autoEndPose == null ? new Pose(8, 8, Math.toRadians(90)) : RobotConfig.autoEndPose);
        follower.update();

        if (RobotConfig.current_color == null) {
            RobotConfig.current_color = RobotConfig.ALLIANCE_COLOR.BLUE;
        }

        this.targetPose = RobotConfig.current_color == RobotConfig.ALLIANCE_COLOR.BLUE
                ? FieldConfig.TARGET_POS_BLUE.copy()
                : FieldConfig.TARGET_POS_RED.copy();
    }

    @Override
    public void periodic() {
        follower.update();
        currentPose = follower.getPose();
        realTurretPose = computeOffset(currentPose, RobotConfig.turret_offset_inches); // get turret pose in robot coordinates

        x = currentPose.getX(); // robot x position
        y = currentPose.getY(); // robot y position

        Pose shiftedTargetPose = getShiftedTargetPose(targetPose, realTurretPose, follower.getVelocity());
        updateShooterSetpoint(targetPose, realTurretPose);

    }

    private Pose getShiftedTargetPose(Pose baseTarget, Pose releasePose, Vector robotVelocity) {
        double dist = releasePose.distanceFrom(baseTarget);
        double time = ShooterLUTs.getTOF(dist);

        double shiftX = robotVelocity.getXComponent() * time;
        double shiftY = robotVelocity.getYComponent() * time;
        return new Pose(baseTarget.getX() + shiftX, baseTarget.getY() + shiftY);
    }

    private void updateShooterSetpoint(Pose shotTarget, Pose releasePose) {
        double dist = releasePose.distanceFrom(shotTarget);

        double hood = ShooterLUTs.getHood(dist);
        double flywheel = ShooterLUTs.getFlywheel(dist);

        double distanceX = shotTarget.getX() - releasePose.getX();
        double distanceY = shotTarget.getY() - releasePose.getY();

        // follower heading assumed radians
        double heading = follower.getTotalHeading(); // robot heading in field frame (rad)


        double fieldAngleRad = Math.atan2(distanceY, distanceX);
        // direction from robot  goal in field frame (rad)

        double turret = UtilMethods.angleWrapRad(fieldAngleRad - heading);
        // desired turret angle relative to robot frame (rad)

        setpoint.hood = hood;
        setpoint.flywheel = flywheel;
        setpoint.turret = turret;
        setpoint.inRange = ShooterLUTs.inRange(dist);
    }

    public static final class ShooterSetpoint {
        double flywheel; // servo ticks per second
        double hood;     // servo position
        double turret;   // motor position
        boolean inRange; // shot is valid

        public ShooterSetpoint(double flywheel, double hood, double turret, boolean inRange) {
            this.flywheel = flywheel;
            this.hood = hood;
            this.turret = turret;
            this.inRange = inRange;
        }
    }

    public double getDist() {
        return realTurretPose.distanceFrom(targetPose);
    }

    public double getHood() {
        return setpoint.hood;
    }

    public double getFlywheel() {
        return setpoint.flywheel;
    }

    public double getTurret() {
        return setpoint.turret;
    }

    public boolean shotAllowed() {
        return setpoint.inRange && (inCloseZone() || inFarZone());
    }

    public void teleOpDrive(double axial, double lateral, double yaw) {
        follower.setTeleOpDrive(
                -axial,
                -lateral,
                -yaw,
                true);
    }

    public Pose computeOffset(Pose pose, double offset) {
        double heading = pose.getHeading();
        double NewX = pose.getX() + Math.cos(heading) * offset;
        double NewY = pose.getY() + Math.sin(heading) * offset;
        return new Pose(NewX, NewY);
    }

    public void reloc(Pose reloc) {
        follower.setPose(reloc);
    }

    public void relocHeading(Pose reloc) {
        follower.setPose(new Pose(x, y, UtilMethods.snapToCardinal(currentPose.getHeading())));
    }


    public Pose getCurrentPose() {
        return currentPose.copy();
    }

    public Pose getTargetPose() {
        return targetPose.copy();
    }

    public boolean inCloseZone() {
        return (y > Math.abs(x - 72) + 72 - FieldConfig.ZONE_BUFFER);
    }

    public boolean inFarZone() {
        return (y < -Math.abs(x - 72) + 24 + FieldConfig.ZONE_BUFFER);
    }
}
