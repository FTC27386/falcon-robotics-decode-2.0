package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPathSlow;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.idleIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Mechanisms.V2CloseZonePaths;
import org.firstinspires.ftc.teamcode.Mechanisms.V2Paths;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Close Auto Blue Gate Cycle")
public class closeZoneAutoBlueGateV2 extends CommandOpMode {
    Follower follower;
    private Robot r;
    V2Paths paths;

    @Override
    public void initialize()
    {
        super.reset();
        RobotConfig.setCurrentColor(RobotConfig.ALLIANCE_COLOR.BLUE);
        r = new Robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Paths.startingPose);
        follower.update();
        paths = new V2Paths(follower);
        register(r.getS(), r.getG(), r.getI());
        //schedule(new RunCommand(() -> r.setShooterValues()));
        schedule(
                new followPath(r, paths.Path1),
                new WaitCommand(500),
                new followPath(r, paths.Path2), //intake 1st line
                new WaitCommand(500),
                new followPath(r, paths.Path3),
                new WaitCommand(500),
                new followPath(r, paths.Path4),
                new WaitCommand(500),
                new followPath(r, paths.Path5),
                new WaitCommand(500),
                new followPath(r, paths.Path6),
                new WaitCommand(500),
                new followPath(r, paths.Path7),
                new WaitCommand(500),
                new followPath(r, paths.Path8),
                new WaitCommand(500),
                new followPath(r, paths.Path9),
                new WaitCommand(500),
                new followPath(r, paths.Path10),
                new WaitCommand(500),
                new followPath(r, paths.Path11),
                new WaitCommand(500)
        );
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
        telemetry.addData("in zone?", r.getD().inCloseZone());
        telemetry.update();
    }
}
