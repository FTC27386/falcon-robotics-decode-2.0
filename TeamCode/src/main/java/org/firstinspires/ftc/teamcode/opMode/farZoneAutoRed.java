package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.autoFarShotRed;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPathSlow;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.idleIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.PathsMirrored;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far Auto Red")
public class farZoneAutoRed extends CommandOpMode {
    Follower follower;
    private Robot r;
    PathsMirrored mirroredPaths;

    @Override
    public void initialize()
    {
        super.reset();

        r = new Robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PathsMirrored.startingPoseFarZone);
        follower.update();
        mirroredPaths = new PathsMirrored(follower);
        register(r.getS(), r.getI());

        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(()-> r.setAutoValuesRed()),
                        new followPath(r, mirroredPaths.farAutoStartPath),
                        new autoFarShotRed(r),
                        new runIntake(r),
                        new followPath(r, mirroredPaths.prepareIntakeHPZonePath),
                        new followPathSlow(r, mirroredPaths.intakeHPZonePath),
                        new ParallelCommandGroup(
                                new followPath(r, mirroredPaths.returnFromHPZonePath),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new idleIntake(r)
                                )
                        ),
                        new autoFarShotRed(r),
                        new followPath(r, mirroredPaths.farLeavePath)
                )
        );
    }
    @Override
    public void run()
    {
        super.run();
        RobotConstants.setCurrent_color(RobotConstants.ALLIANCE_COLOR.RED);
        RobotConstants.setAutoEndPose(r.getD().getCurrentPose());
        telemetry.addData("turretPose",r.getS().getTurretPosition());
        telemetry.addData("robot X", r.getD().getCurrentPose().getX());
        telemetry.addData("robot Y", r.getD().getCurrentPose().getY());
        telemetry.addData("robot heading", Math.toDegrees(r.getD().getCurrentPose().getHeading()));
        telemetry.addData("target X",r.getD().getTargetPose().getX());
        telemetry.addData("target Y",r.getD().getTargetPose().getY());
        telemetry.update();
    }
}
