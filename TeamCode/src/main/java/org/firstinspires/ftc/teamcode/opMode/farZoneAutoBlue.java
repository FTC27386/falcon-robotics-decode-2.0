package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.BOPBOPBOP;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPathSlow;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.idleIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far Auto Blue")
public class farZoneAutoBlue extends CommandOpMode {
    Follower follower;
    private Robot r;
    Paths paths;

    @Override
    public void initialize()
    {
        super.reset();

        r = new Robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Paths.startingPoseFarZone);
        follower.update();
        paths = new Paths(follower);
        register(r.getS(), r.getI());

        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(()-> r.setShooterValues()),
                        new followPath(r, paths.farAutoStartPath),
                        new BOPBOPBOP(r),
                        new runIntake(r),
                        new followPath(r, paths.prepareIntakeHPZonePath),
                        new followPathSlow(r, paths.intakeHPZonePath),
                        new ParallelCommandGroup(
                                new followPath(r, paths.returnFromHPZonePath),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new idleIntake(r)
                                )
                        ),
                        new BOPBOPBOP(r),
                        /*new runIntake(r),
                        new followPath(r, paths.prepareIntakeHPZonePath),
                        new followPathSlow(r, paths.intakeHPZonePath),
                        new ParallelCommandGroup(
                                new followPath(r, paths.returnFromHPZonePath),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new idleIntake(r)
                                )
                        ),*/
                        new followPath(r, paths.farLeavePath)
                )
        );
    }
    @Override
    public void run()
    {
        super.run();
        RobotConfig.setCurrentColor(RobotConfig.ALLIANCE_COLOR.BLUE);
        RobotConfig.setAutoEndPose(r.getD().getCurrentPose());
        telemetry.addData("turretPose",r.getS().getTurretPosition());
        telemetry.addData("robot X", r.getD().getCurrentPose().getX());
        telemetry.addData("robot Y", r.getD().getCurrentPose().getY());
        telemetry.addData("robot heading", Math.toDegrees(r.getD().getCurrentPose().getHeading()));
        telemetry.addData("target X",r.getD().getTargetPose().getX());
        telemetry.addData("target Y",r.getD().getTargetPose().getY());
        telemetry.update();
    }
}
