package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.stopIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Mechanisms.V2Paths;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Close Auto Blue Prototype")
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
        follower.setStartingPose(V2Paths.startingPose);
        follower.update();
        paths = new V2Paths(follower);
        register(r.getD(), r.getS(), r.getG(), r.getI());
        schedule(
                new SequentialCommandGroup
                        (
                                new FollowPathCommand(r.getD().follower, paths.closeAutoStartPath),     // drive to shoot
                                new magDump(r),
                                new WaitCommand(1000),
                                new runIntake(r),                                                       // run intake
                                new FollowPathCommand(r.getD().follower, paths.intakeSecondRowPath),    // intake 2nd line
                                new stopIntake(r),                                                      // stop intake
                                new FollowPathCommand(r.getD().follower, paths.returnToShootPath),      // drive to shoot
                                new magDump(r),
                                new WaitCommand(1000),
                                new FollowPathCommand(r.getD().follower,  paths.openGatePath,true),
                                new WaitCommand(1000),
                                new runIntake(r),
                                new FollowPathCommand(r.getD().follower, paths.intakeFromGatePath, true),
                                new stopIntake(r),                                                          // stop intake
                                new FollowPathCommand(r.getD().follower, paths.returnFromGateToShootPath), // drive to shoot
                                new magDump(r),
                                new WaitCommand(1000),
                                new runIntake(r), // start intake
                                new FollowPathCommand(r.getD().follower, paths.intakeFirstRowPath), // intake first row
                                new stopIntake(r), // stop intake
                                new FollowPathCommand(r.getD().follower, paths.returnFromIntakeFirstToShootPath), // drive to shoot
                                new magDump(r), // shoot
                                new WaitCommand(1000),
                                new runIntake(r),
                                new FollowPathCommand(r.getD().follower, paths.intakeThirdRowPath),
                                new stopIntake(r),
                                new FollowPathCommand(r.getD().follower, paths.returnFromThirdRowToShootPath),
                                new magDump(r),
                                new WaitCommand(1000),
                                new FollowPathCommand(r.getD().follower, paths.parkPath)
                        )
        );
        schedule(new RunCommand(() -> r.setShooterValues()));
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
        telemetry.addData("current path to follow", r.getD().follower.getCurrentPathChain());
        telemetry.update();
    }
}
