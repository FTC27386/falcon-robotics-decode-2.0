package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPathSlow;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.idleIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Mechanisms.V2Paths;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Nothing auto")
public class nothingAuto extends CommandOpMode {
    Follower follower;
    public static Robot r;
    V2Paths paths;

    @Override
    public void initialize()
    {
        super.reset();
        RobotConfig.setCurrentColor(RobotConfig.ALLIANCE_COLOR.BLUE);
        r = new Robot(hardwareMap);
        RobotConfig.setCurrentRobotInstance(r);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,Math.toRadians(90)));
        follower.update();
        paths = new V2Paths(follower, RobotConfig.current_color);
        register(r.getS(), r.getG(), r.getI());
        schedule(new RunCommand(() -> r.setShooterValues()));
        schedule(new RunCommand(() -> r.getD().updateTargetAndRelocPose()));
    }
    @Override
    public void run() {
        super.run();
        RobotConfig.setAutoEndPose(r.getD().getCurrentPose());
        telemetry.addData("turretPose",r.getS().getTurretTarget());
        telemetry.addData("robot X", r.getD().getCurrentPose().getX());
        telemetry.addData("robot Y", r.getD().getCurrentPose().getY());
        telemetry.addData("robot heading", Math.toDegrees(r.getD().getCurrentPose().getHeading()));
        telemetry.addData("target X",r.getD().getTargetPose().getX());
        telemetry.addData("target Y",r.getD().getTargetPose().getY());
        telemetry.addData("in zone?", r.getD().inCloseZone());
        telemetry.addData("turret target", r.getS().getTurretTarget());
        telemetry.addData("turret current", r.getS().getTurretAngle());
        telemetry.addData("turret error", r.getS().getError());
        telemetry.addData("current path to follow", r.getD().follower.getCurrentPathChain());
        telemetry.update();
    }
}
