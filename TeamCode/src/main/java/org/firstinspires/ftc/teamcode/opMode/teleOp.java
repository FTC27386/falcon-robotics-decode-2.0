package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorLimelight3A;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.defaultDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeReverseTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.PathsMirrored;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;

import java.util.function.Supplier;

@Config
@TeleOp(name = "TeleOp")
public class teleOp extends CommandOpMode {
    Button intake;
    Button relocalize;
    Button shoot;
    Button outtake;
    Button changeTarget;
    GamepadEx driverOp;
    GamepadEx driver2Op;
    Button lift;
    Button stop;
    private Robot r;
    SlewRateLimiter axial;

    @Override
    public void initialize() {
        axial = new SlewRateLimiter(0.3, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        super.reset();
        r = new Robot(hardwareMap);
        register(r.getV(), r.getD(), r.getS(), r.getG(), r.getI(), r.getL());
        driverOp = new GamepadEx(gamepad1);
        driverOp.setJoystickSlewRateLimiters(null, axial, null, null);

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
        lift = driverOp.getGamepadButton(GamepadKeys.Button.TOUCHPAD);
        stop = driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP);

        lift.whenPressed(new followPath(r,
                RobotConfig.current_color == null || RobotConfig.current_color == RobotConfig.ALLIANCE_COLOR.RED?
                        paths.park : mirroredPaths.park));
        intake.whenPressed(new runIntakeTimed(r, 2000));
        outtake.whenPressed(new runIntakeReverseTimed(r, 2000));
        relocalize.whenPressed(new InstantCommand(() -> r.getD().reloc(new Pose(8, 8, Math.toRadians(90)))));
        schedule(new InstantCommand(() -> r.getD().follower.startTeleOpDrive()));
        schedule(new RunCommand(() -> r.setShooterValues()));
        shoot.toggleWhenPressed(new magDump(r));
    }

    @Override
    public void run() {

        telemetry.addData("flywheel target velocity", r.getD().getFlywheel());
        telemetry.addData("hood", r.getD().getHood());
        telemetry.addData("distance", r.getD().getDist());
        telemetry.addData("x:", r.getD().getCurrentPose().getX());
        telemetry.addData("y:", r.getD().getCurrentPose().getY());
        telemetry.addData("in zone?", r.getD().shotAllowed());
        telemetry.update();
        super.run();
    }

}
