package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.Utility.RobotConstants.zone_buffer;

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
    private final InterpLUT closeLUT = new InterpLUT();
    private final InterpLUT farLUT = new InterpLUT();

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
            targetPose = new Pose(5, 139);
        } else {
            targetPose = new Pose(139, 139);
        }

        //Init the Look up table
        closeLUT.add(35.00, 0.028);
        closeLUT.add(38.48, 0.038);
        closeLUT.add(41.12, 0.055);
        closeLUT.add(44.76, 0.12);
        closeLUT.createLUT();

        farLUT.add(44.76, 0.08);
        farLUT.add(44.77, 0.08);
        farLUT.add(48.40, 0.082);
        farLUT.add(51.31, 0.12);
        farLUT.add(52.31, 0.13);
        farLUT.add(54.08, 0.15);
        farLUT.add(56.04, 0.153);
        farLUT.add(58.15, 0.17);
        farLUT.add(62.02, 0.19);
        farLUT.add(67.81, 0.3);
        farLUT.createLUT();
    }

    @Override
    public void periodic() {
        follower.update();
        currentPose = follower.getPose();
        calculatedPose = getPredictedPose(1);
        realTurretPose = computeOffset(currentPose, RobotConstants.turret_offset_inches);

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
        if (dist > 35.00 && dist < 44.76) {
            return closeLUT.get(dist);
        }
        else if (dist >= 44.76 && dist < 67.81) {
            return farLUT.get(dist);
        }
        else return 0;
    }

    public double getSpeed() {
        dist = getDist();
        if (dist > 35.00 && dist < 44.76) {
            return -1300;
        }
        else if (dist >= 44.76 && dist < 67.81) {
            return -1450;
        }
        else return 0;
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
        return (y > Math.abs(x - 72) + 72 - zone_buffer);
    }
    public boolean inFarZone() {
        return (y < -Math.abs(x - 72) + 24 + zone_buffer);
    }
}
