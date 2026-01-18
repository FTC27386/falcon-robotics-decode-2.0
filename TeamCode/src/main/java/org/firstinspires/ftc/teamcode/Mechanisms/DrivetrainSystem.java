package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.Utility.RobotConstants.zone_buffer;
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
import org.firstinspires.ftc.teamcode.Utility.UtilMethods;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

public class DrivetrainSystem extends SubsystemBase {
    public Pose
            currentPose = new Pose(0, 0, Math.toRadians(90)),
            calculatedPose,
            realTurretPose = new Pose(0,0),
            targetPose;
    public Follower follower;
    private final InterpLUT LUT1 = new InterpLUT();
    private final InterpLUT LUT2 = new InterpLUT();
    private final InterpLUT LUT3 = new InterpLUT();
    public boolean shooterOff = false;

    public double
            x,
            y,
            distanceX,
            distanceY,
            dist,
            heading,
            unnormalizedHeading,
            field_angle,
            other_distance;
    public Supplier<Pose> poseSupplier = this::getCurrentPose;
    boolean robotCentricDrive = false;

    public DrivetrainSystem(HardwareMap hMap) {
        follower = Constants.createFollower(hMap);
        follower.setStartingPose(RobotConstants.autoEndPose == null ? new Pose(8, 8, Math.toRadians(90)) : RobotConstants.autoEndPose);
        follower.update();
        if (RobotConstants.current_color == null || RobotConstants.current_color == RobotConstants.ALLIANCE_COLOR.BLUE) {
            targetPose = new Pose(0, 144);
        } else {
            targetPose = new Pose(144, 144);
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
        dist = getDist();
        if      (dist >= 45.00 && dist <  65.00) return LUT1.get(dist);
        else if (dist >= 65.00 && dist <  85.00) return LUT2.get(dist);
        else if (dist >= 85.00 && dist <= 95.00) return LUT3.get(dist);
        else                                     return 0;
    }

    public double getSpeed() {
        dist = getDist();
        if (shooterOff) return 0;
        if      (dist >= 45.00 && dist <  65.00) return -1400;
        else if (dist >= 65.00 && dist <  85.00) return -1500;
        else if (dist >= 85.00 && dist <= 95.00) return -1600;
        else                                     return -1600;
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
