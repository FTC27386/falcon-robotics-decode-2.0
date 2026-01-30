package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.BOPBOPBOP;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPathSlow;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeReverseTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.idleIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.PathsMirrored;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Close Auto Red")
public class closeZoneAutoRed extends CommandOpMode {
    Follower follower;
    private Robot r;
    PathsMirrored paths;

    @Override
    public void initialize()
    {
        super.reset();
        RobotConfig.setCurrentColor(RobotConfig.ALLIANCE_COLOR.RED);
        r = new Robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PathsMirrored.startingPose);
        follower.update();
        paths = new PathsMirrored(follower);
        register(r.getS(), r.getI());
        schedule(new RunCommand(()->r.setShooterValues()));
        schedule(new InstantCommand(()->r.getS().setFlywheelSpeed(-1570)));
        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(()-> r.getS().setGate(false)),
                        new followPath(r, paths.closeAutoStartPath),
                        new BOPBOPBOP(r),
                        new runIntakeReverseTimed(r, 100),
                        new runIntake(r),
                new followPathSlow(r, paths.intakeFirstRowPath), //intake 1st line
                new ParallelCommandGroup(
                                new followPath(r,paths.returnFromTopRowPath),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                                new idleIntake(r))

                ),
                new BOPBOPBOP(r),
                new runIntakeReverseTimed(r, 100),
                new followPath(r, paths.prepareIntakeMiddleRowPath),
                new runIntake(r),
                new followPathSlow(r, paths.intakeMiddleRowPath),
                new ParallelCommandGroup(
                        new followPath(r,paths.returnFromMiddleRowPath),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new idleIntake(r))
                ),
                new BOPBOPBOP(r),
                new runIntakeReverseTimed(r, 100),
                /*
                new followPath(r, paths.prepareIntakeBottomRowPath),
                new runIntake(r),
                new followPathSlow(r, paths.intakeBottomRowPath),
                new ParallelCommandGroup(
                        new followPath(r,paths.returnFromBottomRowPath),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new idleIntake(r))
                ),
                new BOPBOPBOP(r),
                new runIntakeReverseTimed(r, 100),

                 */
                new followPath(r, paths.goToGatePath)));
    }
    @Override
    public void run()
    {
        super.run();
        RobotConfig.setAutoEndPose(r.getD().getCurrentPose());
        telemetry.addData("turretPose",r.getS().getTurretPosition());
        telemetry.addData("robot X", r.getD().getCurrentPose().getX());
        telemetry.addData("robot Y", r.getD().getCurrentPose().getY());
        telemetry.addData("robot heading", Math.toDegrees(r.getD().getCurrentPose().getHeading()));
        telemetry.addData("target X",r.getD().getTargetPose().getX());
        telemetry.addData("target Y",r.getD().getTargetPose().getY());
        telemetry.addData("in zone?", r.getD().inCloseZone());
        telemetry.update();
    }
}
