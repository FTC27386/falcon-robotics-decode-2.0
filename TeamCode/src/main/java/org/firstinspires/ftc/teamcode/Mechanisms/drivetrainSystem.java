package org.firstinspires.ftc.teamcode.Mechanisms;


import static org.firstinspires.ftc.teamcode.opMode.teleOp.flywheel_speed;
import static org.firstinspires.ftc.teamcode.opMode.teleOp.hood_pos;
import static java.lang.Math.sqrt;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;
import org.firstinspires.ftc.teamcode.Utility.UtilMethods;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

public class drivetrainSystem extends SubsystemBase {
    public static Pose
            currentPose = new Pose(0, 0, Math.toRadians(90)),
            calculatedPose,
            startPose,
            targ;
    public Follower follower;

    public double
            x,
            y,
            distanceX,
            distanceY,
            heading,
            unnormalizedHeading,
            field_angle,
            zoneBuffer = 7.5 * Math.sqrt(2),
            act_x,
            act_y,
            act_distanceX,
            act_distanceY;
    public Supplier<Pose> poseSupplier = this::getCurrentPose;
    boolean robotCentricDrive = false;

    public drivetrainSystem(HardwareMap hMap) {
        follower = Constants.createFollower(hMap);
        follower.setStartingPose(RobotConstants.autoEndPose == null ? new Pose(8, 8, Math.toRadians(90)) : RobotConstants.autoEndPose);
        follower.update();
        if (RobotConstants.current_color == null || RobotConstants.current_color == RobotConstants.ALLIANCE_COLOR.BLUE) {
            targ = new Pose(0, 144);
        } else {
            targ = new Pose(144, 144);
        }
    }

    @Override
    public void periodic() {
        follower.update();
        currentPose = follower.getPose();
        calculatedPose = getPredictedPose(1);

        x = currentPose.getX();
        y = currentPose.getY();

        distanceX = targ.getX() - x;
        distanceY = targ.getY() - y;

        /*
        x = calculatedPose.getX();
        y = calculatedPose.getY();

        act_x = currentPose.getX();
        act_y = currentPose.getY();

        distanceX = targ.getX() - x;
        distanceY = targ.getY() - y;

        act_distanceX = targ.getX() - act_x;
        act_distanceY = targ.getY() - act_y;
         */

        heading = currentPose.getHeading();
        unnormalizedHeading = follower.getTotalHeading();
    }
    public double yoCalcDist() { return Math.hypot(distanceX, distanceY); }
    //public double yoCalcActDist() { return Math.hypot(act_distanceX, act_distanceY); }
    public double yoCalcAim()  //calculate adjusted turret angle in degrees
    {
        field_angle = (90 - Math.toDegrees(Math.atan2(distanceY, distanceX)));
        // return Math.toDegrees((heading)-Math.PI/2) + field_angle;
        return -UtilMethods.AngleDifference(Math.toDegrees(unnormalizedHeading), 0) + field_angle;

        //equivalent: Math.toDegrees(currentpose.getHeading() - Math.PI/2)

        //(90 - Math.toDegrees(Math.atan2(distanceY,distanceX)))
    }

    public double yoCalcHood() {
        /*
        dist = yoCalcDist();
        if (dist >= 50.2778 && dist <= 89.5204) {
            return (0.00000106851 * Math.pow(dist, 4))
                    - (0.000298793 * Math.pow(dist, 3))
                    + (0.0307389 * Math.pow(dist, 2))
                    - (1.37843 * dist)
                    + 23.22303;
        } else if (dist > 89.5204 && dist <= 110.2215) {
            return (-0.0000318774 * Math.pow(dist, 3))
                    + (0.00925592 * Math.pow(dist, 2))
                    - (0.876897 * dist)
                    + 27.89043;
        } else {
            return 0.5;
        }
         */
        
        return 0.35;
    }

    public double yoCalcSpeed() {
        /*
        dist = yoCalcDist();
        if (dist >= 50.2778 && dist <= 89.5204) {
            return -1700;
        } else if (dist > 89.5204 && dist <= 110.2215) {
            return -2000;
        } else {
            return 0;
        }
         */
        return -6.69464 * yoCalcDist() - 967.24813;
    }

    public void teleOpDrive(double axial, double lateral, double yaw) {
        follower.setTeleOpDrive(
                -axial,
                -lateral,
                -yaw,
                true);
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

    public Pose getTarg() {
        return targ;
    }

    public void relocTarget(Pose reloc) {
        targ = reloc;
    }

    public boolean inZone() {
        return true;
        //return (y > Math.abs(x - 72) + 72 - zoneBuffer) || (y < -Math.abs(x - 72) + 24 + zoneBuffer);
    }
}
