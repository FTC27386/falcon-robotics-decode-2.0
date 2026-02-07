package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPathSlow;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.idleIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.FZPaths;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far Auto Blue V2")
public class farZoneAutoRed extends CommandOpMode {
    Follower follower;
    private Robot r;
    FZPaths paths;

    @Override
    public void initialize()
    {
        super.reset();

        r = new Robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(FZPaths.initPose.mirror());
        follower.update();
        paths = new FZPaths(follower, RobotConfig.ALLIANCE_COLOR.RED);
        register(r.getS(), r.getG(), r.getI());
        schedule(new RunCommand(()->r.getS().setTurretAngle(r.getD().getTurret())));
        schedule(
                new SequentialCommandGroup(
                        new followPath(r, paths.intakeHP),
                        new followPath(r, paths.intakeHPreturn),
                        new followPath(r, paths.intake3rdSpikeA),
                        new followPath(r, paths.intake3rdSpikeB),
                        new followPath(r, paths.return3rdSpike),
                        new followPath(r, paths.blindIntake),
                        new followPath(r, paths.blindIntakeReturn)
                )
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
        telemetry.update();
    }
}
