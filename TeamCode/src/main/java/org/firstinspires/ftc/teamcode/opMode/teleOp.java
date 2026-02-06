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

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.defaultDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.liftoff;
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
    Button outtake;
    Button relocalize;
    Button shoot;
    Button climb;
    Button stop;
    Button increaseTurretOffset;
    Button decreaseTurretOffset;
    Button changeTarget;
    GamepadEx driverOp;
    GamepadEx driver2Op;
    private Robot r;
    public SlewRateLimiter axial;

    @Override
    public void initialize() {
        axial = new SlewRateLimiter(12,-1000,0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        super.reset();
        r = new Robot(hardwareMap);
        register(r.getV(), r.getD(), r.getS(), r.getG(), r.getI(), r.getL());
        driverOp = new GamepadEx(gamepad1);
        driverOp.setJoystickSlewRateLimiters(null,axial,null,null);
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
        shoot = driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        stop = driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        climb = driverOp.getGamepadButton(GamepadKeys.Button.TOUCHPAD);
        decreaseTurretOffset = driver2Op.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        increaseTurretOffset = driver2Op.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);

        intake.whenPressed(new runIntakeTimed(r, 2000));
        outtake.whenPressed(new runIntakeReverseTimed(r, 2000));
        relocalize.whenPressed(new InstantCommand(() -> r.getD().reloc()));
        shoot.whenPressed(new magDump(r));
        stop.whenPressed(new followPath(r,
                RobotConfig.current_color == null || RobotConfig.current_color == RobotConfig.ALLIANCE_COLOR.RED?
                        paths.park : mirroredPaths.park));
        climb.whenPressed(new liftoff(r));
        decreaseTurretOffset.whenPressed(new InstantCommand(() -> r.getD().decreaseOffset()));
        increaseTurretOffset.whenPressed(new InstantCommand(() -> r.getD().increaseOffset()));
        schedule(new InstantCommand(() -> r.getD().follower.startTeleOpDrive()));
        schedule(new RunCommand(() -> r.setShooterValues()));
    }

    @Override
    public void run() {
        telemetry.addData("flywheel power", r.getS().getFlywheelPower());
        telemetry.addData("flywheel speed", r.getS().getFlywheelSpeed());
        telemetry.addData("flywheel target", r.getS().getFlywheelTarget());
        telemetry.addData("turret power", r.getS().getTurretPower());
        telemetry.addData("turret angle", r.getS().getTurretAngle());
        telemetry.addData("turret target", r.getS().getTurretTarget());
        telemetry.addData("hood", r.getS().getHoodAngle());
        telemetry.addData("x:", r.getD().getCurrentPose().getX());
        telemetry.addData("y:", r.getD().getCurrentPose().getY());
        telemetry.addData("distance", r.getD().getDist());
        telemetry.addData("heading", r.getD().getCurrentPose().getHeading());
        telemetry.addData("error of turret", r.getS().getError());
        telemetry.addData("valid shot", r.getD().shotAllowed());

        telemetry.update();
        super.run();
    }
}
