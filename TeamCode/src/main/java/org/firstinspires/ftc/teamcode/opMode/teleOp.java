package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.defaultDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.liftoff;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeReverseTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.PathsMirrored;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

import java.util.function.Supplier;

@Config
@TeleOp(name = "TeleOp")
public class teleOp extends CommandOpMode {
    ElapsedTime timer;
    double loop_time;
    Button intake;
    Button relocalize;
    Button shoot;
    Button outtake;
    Button changeTarget;
    GamepadEx driverOp;
    Button climb;
    Button park;
    public static double hood_pos = 0;
    public static double flywheel_speed = -1700;
    private Robot r;
    Paths paths;
    PathsMirrored paths_mirrored;

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        super.reset();
        r = new Robot(hardwareMap);
        register(r.getS(), r.getD(), r.getI(), r.getL());
        driverOp = new GamepadEx(gamepad1);
        Supplier<Double> leftX = driverOp::getLeftX;
        Supplier<Double> leftY = driverOp::getLeftY;
        Supplier<Double> rightX = driverOp::getRightX;
        r.getD().setDefaultCommand(new defaultDrive(r, leftY, leftX, rightX));
        Paths paths = new Paths(r.getD().follower);
        PathsMirrored mirroredPaths = new PathsMirrored(r.getD().follower);


        intake = driverOp.getGamepadButton(GamepadKeys.Button.SQUARE);
        outtake = driverOp.getGamepadButton(GamepadKeys.Button.CIRCLE);
        relocalize = driverOp.getGamepadButton(GamepadKeys.Button.OPTIONS);
        changeTarget = driverOp.getGamepadButton(GamepadKeys.Button.SHARE);
        shoot = driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        climb = driverOp.getGamepadButton(GamepadKeys.Button.TOUCHPAD);
        park = driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP);

        climb.whenPressed(new followPath(r,
                RobotConstants.current_color == null || RobotConstants.current_color == RobotConstants.ALLIANCE_COLOR.RED?
                        paths.park : mirroredPaths.park));
        intake.whenPressed(new runIntakeTimed(r, 2000));
        outtake.whenPressed(new runIntakeReverseTimed(r, 2000));
        relocalize.whenPressed(new InstantCommand(() -> r.getD().reloc(new Pose(8, 8, Math.toRadians(90)))));
        changeTarget.whenPressed(new InstantCommand(() -> r.getD().relocTarget(
               new Pose(
                       r.getD().getCurrentPose().getX() - 12
                       ,r.getD().getCurrentPose().getY() + 12
                       ,Math.toRadians(90)
               ))));
        schedule(new InstantCommand(() -> r.getD().follower.startTeleOpDrive()));
        schedule(new RunCommand(() -> r.getS().setTurretPosition(r.getD().yoCalcAim())));
        schedule(new RunCommand(() -> r.getS().setHoodPosition(r.getD().yoCalcHood())));
        schedule(new RunCommand(() -> r.getS().setSpeed(r.getD().yoCalcSpeed())));
        shoot.whenPressed(new magDump(r));
        park.whenPressed(new liftoff(r));
    }

    @Override
    public void run() {
        loop_time = timer.seconds();
        timer.reset();



        telemetry.addData("x:", r.getD().x);
        telemetry.addData("y:", r.getD().y);
        telemetry.addData("Actual x:", r.getD().act_x);
        telemetry.addData("Actual y:", r.getD().act_y);
        telemetry.addData("Distance", r.getD().yoCalcDist());
        telemetry.addData("Vera Distance", r.getD().other_distance);
        //telemetry.addData("Actual Distance", r.getD().yoCalcActDist());
        telemetry.addData("target X", r.getD().getTarg().getX());
        telemetry.addData("target Y", r.getD().getTarg().getY());
        telemetry.addData("heading", r.getD().getCurrentPose().getHeading());
        telemetry.addData("adjustment", r.getD().yoCalcAim());
        telemetry.addData("flywheel target velocity", r.getS().getSpeedControl().getSetPoint());
        telemetry.addData("flywheel error", r.getS().getSpeedControl().getPositionError());
        telemetry.addData("flywheel speed", r.getS().getCurrentSpeed());
        telemetry.addData("Hood", r.getD().yoCalcHood());
        telemetry.addData("flywheel response", r.getS().getFlywheelSignal());
        telemetry.addData("turret ticks", r.getS().getTurretPosition());
        telemetry.addData("lift power", r.getL().getPIDResponse());
        telemetry.addData("lift pose", r.getL().getLiftPose());
        telemetry.addData("In zone", r.getD().inZone());
        telemetry.addData("alliance color?", RobotConstants.current_color);
        telemetry.addData("loop time in ms", loop_time*1000);
        telemetry.addData("loop frequency", Math.pow((loop_time),-1));
        telemetry.addData("turret X", r.getD().realTurretPose.getX());
        telemetry.addData("turret Y", r.getD().realTurretPose.getY());
        telemetry.update();
        super.run();
    }

}
