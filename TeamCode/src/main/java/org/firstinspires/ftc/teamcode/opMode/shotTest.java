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

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.BOPBOPBOP;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.defaultDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.liftoff;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.manualTriple;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeReverseTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.PathsMirrored;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

import java.util.function.Supplier;

@Config
@TeleOp(name = "Shot Test")
public class shotTest extends CommandOpMode {
    ElapsedTime timer;
    double loop_time;
    Button intake;
    Button relocalize;
    Button shoot;
    Button outtake;
    Button changeTarget;
    GamepadEx driverOp;
    GamepadEx driver2Op;
    Button climb;
    Button park;
    Button increaseOffset;
    Button decreaseOffset;
    Button increaseAOA;
    Button decreaseAOA;
    private Robot r;
    Paths paths;
    PathsMirrored paths_mirrored;
    public static double flywheel_speed;
    public static double hood_angle;

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        super.reset();
        r = new Robot(hardwareMap);
        register(r.getS(), r.getD(), r.getI(), r.getL(), r.getV());
        driverOp = new GamepadEx(gamepad1);
        driver2Op = new GamepadEx(gamepad2);
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
        increaseOffset = driver2Op.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        decreaseOffset = driver2Op.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);


        climb.whenPressed(new followPath(r,
                RobotConstants.current_color == null || RobotConstants.current_color == RobotConstants.ALLIANCE_COLOR.RED?
                        paths.park : mirroredPaths.park));
        intake.whenPressed(new runIntakeTimed(r, 2000));
        outtake.whenPressed(new runIntakeReverseTimed(r, 2000));
        relocalize.whenPressed(new InstantCommand(() -> r.getD().reloc(new Pose(8, 8, Math.toRadians(90)))));
        changeTarget.whenPressed(new InstantCommand(() -> r.getD().relocTargetPose(
               new Pose(
                       r.getD().getCurrentPose().getX() - 12
                       ,r.getD().getCurrentPose().getY() + 12
                       ,Math.toRadians(90)
               ))));
        schedule(new InstantCommand(() -> r.getD().follower.startTeleOpDrive()));
        schedule(new RunCommand(() -> r.getS().setTurretPosition(r.getD().getAim())));
        //schedule(new RunCommand(() -> r.getV().startLimelight(telemetry)));
        shoot.whenPressed(new manualTriple(r, flywheel_speed, hood_angle));
        park.whenPressed(new liftoff(r));
        increaseOffset.whenPressed(new InstantCommand(()->r.getS().nudgeOffset(-4)));
        decreaseOffset.whenPressed(new InstantCommand(()->r.getS().nudgeOffset(4)));
    }

    @Override
    public void run() {
        loop_time = timer.seconds();
        timer.reset();

        /*
        telemetry.addData("x:", r.getD().x);
        telemetry.addData("y:", r.getD().y);
        telemetry.addData("Distance", r.getD().getDist());
        telemetry.addData("target X", r.getD().getTargetPose().getX());
        telemetry.addData("target Y", r.getD().getTargetPose().getY());
        telemetry.addData("heading", r.getD().getCurrentPose().getHeading());
        telemetry.addData("adjustment", r.getD().getAim());
        telemetry.addData("flywheel target velocity", r.getS().getSpeedControl().getSetPoint());
        telemetry.addData("flywheel error", r.getS().getSpeedControl().getPositionError());
        telemetry.addData("flywheel speed", r.getS().getCurrentSpeed());
        telemetry.addData("Hood", r.getD().getHood());
        telemetry.addData("flywheel response", r.getS().getFlywheelSignal());
        telemetry.addData("turret ticks", r.getS().getTurretPosition());
        telemetry.addData("lift power", r.getL().getPIDResponse());
        telemetry.addData("lift pose", r.getL().getLiftPose());
        telemetry.addData("In close zone", r.getD().inCloseZone());
        telemetry.addData("In far zone", r.getD().inFarZone());
        telemetry.addData("alliance color?", RobotConstants.current_color);
        telemetry.addData("loop time in ms", loop_time*1000);
        telemetry.addData("loop frequency", Math.pow((loop_time),-1));
        telemetry.addData("turret X", r.getD().realTurretPose.getX());
        telemetry.addData("turret Y", r.getD().realTurretPose.getY());
        */

        telemetry.addData("flywheel target velocity", r.getS().getSpeedControl().getSetPoint());
        telemetry.addData("current speed", r.getS().getCurrentSpeed());
        telemetry.addData("distance", r.getD().getDist());
        telemetry.addData("hood", r.getD().getHood());
        telemetry.update();
        super.run();
    }

}
