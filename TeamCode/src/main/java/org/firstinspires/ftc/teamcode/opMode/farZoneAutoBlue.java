package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPathSlow;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.idleIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.stopIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.FZPaths;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far Auto Pathfollowing")
public class farZoneAutoBlue extends CommandOpMode {
    Follower follower;
    private Robot r;
    FZPaths paths;

    @Override
    public void initialize()
    {
        super.reset();

        r = new Robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(FZPaths.initPose);
        follower.update();
        paths = new FZPaths(follower, RobotConfig.ALLIANCE_COLOR.BLUE);

        register(r.getS(), r.getG(), r.getI());
        schedule(new RunCommand(()->r.setShooterValues()));

        schedule(
                new SequentialCommandGroup(
                        new magDump(r,-.8),
                        new runIntake(r),
                        new FollowPathCommand(follower, paths.intakeHP),
                        new WaitCommand(250),
                        new stopIntake(r),
                        new FollowPathCommand(follower, paths.intakeHPreturn),
                        new magDump(r,-.8),
                        new FollowPathCommand(follower, paths.intake3rdSpikeA),
                        new FollowPathCommand(follower, paths.intake3rdSpikeB),
                        new WaitCommand(250),
                        new stopIntake(r),
                        new FollowPathCommand(follower, paths.return3rdSpike),
                        new magDump(r,-.8),
                        new runIntake(r),
                        new FollowPathCommand(follower, paths.blindIntake),
                        new WaitCommand(250),
                        new stopIntake(r),
                        new FollowPathCommand(follower, paths.blindIntakeReturn),
                        new magDump(r,-.8)
                )
        );
    }
    @Override
    public void run()
    {
        super.run();
        RobotConfig.setCurrentColor(RobotConfig.ALLIANCE_COLOR.BLUE);
        RobotConfig.setAutoEndPose(r.getD().getCurrentPose());
        telemetry.addData("turretPose",r.getS().getTurretTarget());
        telemetry.addData("robot X", r.getD().getCurrentPose().getX());
        telemetry.addData("robot Y", r.getD().getCurrentPose().getY());
        telemetry.addData("robot heading", Math.toDegrees(r.getD().getCurrentPose().getHeading()));
        telemetry.addData("target X",r.getD().getTargetPose().getX());
        telemetry.addData("target Y",r.getD().getTargetPose().getY());
        telemetry.update();
    }
}
