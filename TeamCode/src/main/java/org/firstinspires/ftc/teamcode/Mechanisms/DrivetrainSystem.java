package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.Utility.RobotConstants.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.Utility.RobotConstants.BLUE_TARGET;
import static org.firstinspires.ftc.teamcode.Utility.RobotConstants.RED_GOAL;
import static org.firstinspires.ftc.teamcode.Utility.RobotConstants.RED_TARGET;
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

public class DrivetrainSystem extends SubsystemBase {
    public Pose
        currentPose = new Pose(0, 0, Math.toRadians(90)),
        realTurretPose = new Pose(0,0),
        targetPose,
        goalPose;

    public Follower follower;
    private final InterpLUT[] LUT = new InterpLUT[3];

    public double
            x,
            y,
            distanceX,
            distanceY,
            dist,
            heading,
            unnormalizedHeading,
            field_angle;
    public DrivetrainSystem(HardwareMap hMap) {
        follower = Constants.createFollower(hMap);
        follower.setStartingPose(RobotConstants.autoEndPose == null ? new Pose(8, 8, Math.toRadians(90)) : RobotConstants.autoEndPose);
        follower.update();
        if (RobotConstants.current_color == null || RobotConstants.current_color == RobotConstants.ALLIANCE_COLOR.BLUE) {
            goalPose = BLUE_GOAL;
            targetPose = BLUE_TARGET;
        } else {
            goalPose = RED_GOAL;
            targetPose = RED_TARGET;
        }

        //-1400
        LUT[0].add(45.00, 0.2);
        LUT[0].add(55.00, 0.25);
        LUT[0].add(65.00, 0.4);
        LUT[0].createLUT();

        //-1500
        LUT[1].add(65.00, 0.25);
        LUT[1].add(75.00, 0.35);
        LUT[1].add(85.00, 0.4);
        LUT[1].createLUT();

        //-1600
        LUT[2].add(85.00, 0.3);
        LUT[2].add(90.00, 0.25);
        LUT[2].add(95.70, 0.3);
        LUT[2].createLUT();
    }

    @Override
    public void periodic() {
        follower.update();
        currentPose = follower.getPose();
        realTurretPose = computeOffset(currentPose, RobotConstants.turret_offset_inches);

        x = currentPose.getX();
        y = currentPose.getY();

        distanceX = targetPose.getX() - realTurretPose.getX();
        distanceY = targetPose.getY() - realTurretPose.getY();

        heading = currentPose.getHeading();
        unnormalizedHeading = follower.getTotalHeading();
    }
    public double getDist() { return follower.getPose().distanceFrom(goalPose); }
    public double getAim()  //calculate adjusted turret angle in degrees
    {
        field_angle = (90 - Math.toDegrees(Math.atan2(distanceY, distanceX)));
        return -UtilMethods.AngleDifference(Math.toDegrees(unnormalizedHeading), 0) + field_angle;
    }

    public double getHood() {
        dist = getDist();
        if      (dist >= 45.00 && dist <  65.00) return LUT[0].get(dist);
        else if (dist >= 65.00 && dist <  85.00) return LUT[1].get(dist);
        else if (dist >= 85.00 && dist <= 95.00) return LUT[2].get(dist);
        else                                     return 0;
    }

    public double getSpeed() {
        dist = getDist();
        if      (dist >= 45.00 && dist <  65.00) return -1400;
        else if (dist >= 65.00 && dist <  85.00) return -1500;
        else if (dist >= 85.00 && dist <= 95.00) return -1600;
        else                                     return -1600;
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
    public Pose AprilTagReloc() {
        //get x and y position of limelight
        //get current robot heading
        //get current turret set heading
        //add to find turret heading
        currentPose.
        return new Pose(x2,y2);
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
