package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.BOPBOPBOP;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.idleIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.stopIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.FZPaths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far Auto Red V2")
public class farZoneAutoRedPreload extends CommandOpMode {
    Follower follower;
    private Robot r;
    FZPaths paths;

    @Override
    public void initialize()
    {
        super.reset();

        r = new Robot(hardwareMap);
        RobotConfig.setCurrentRobotInstance(r);
        RobotConfig.setCurrentColor(RobotConfig.ALLIANCE_COLOR.RED);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(FZPaths.alternatePose);
        follower.update();
        paths = new FZPaths(follower, RobotConfig.ALLIANCE_COLOR.RED);
        register(r.getS(), r.getG(), r.getI());
        schedule(new RunCommand(()->r.setShooterValues()));
        schedule(new RunCommand(()->r.getD().updateTargetAndRelocPose()));
        schedule(
                new SequentialCommandGroup(
                        new WaitCommand(2000),
                        new BOPBOPBOP(r),
                        new runIntake(r),
                        new FollowPathCommand(follower, paths.intakeHP),
                        new WaitCommand(250),
                        new idleIntake(r),
                        new FollowPathCommand(follower, paths.intakeHPreturn),
                        new BOPBOPBOP(r),

                        new runIntake(r),
                        new FollowPathCommand(follower, paths.intake3rdSpikeA),
                        new FollowPathCommand(follower, paths.intake3rdSpikeB),
                        new WaitCommand(250),
                        new idleIntake(r),
                        new FollowPathCommand(follower, paths.return3rdSpike),
                        new BOPBOPBOP(r),

                        new runIntake(r),
                        new FollowPathCommand(follower, paths.blindIntake),
                        new WaitCommand(250),
                        new idleIntake(r),
                        new FollowPathCommand(follower, paths.blindIntakeReturn),
                        new BOPBOPBOP(r)
        ));
    }
    @Override
    public void run()
    {
        super.run();
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
