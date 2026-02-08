package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.BOPBOPBOP;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.defaultDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeReverseTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.stop;
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
    Button d1Relocalize;
    Button d2Relocalize;
    Button shootClose;
    Button shootFar;
    Button lift;
    Button d1Stop;
    Button d2Stop;
    Button increaseTurretOffset;
    Button decreaseTurretOffset;
    GamepadEx driverOp;
    GamepadEx driver2Op;
    private Robot r;
    public SlewRateLimiter axial;

    @Override
    public void initialize() {
        axial = new SlewRateLimiter(12,-1000,0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        super.reset();
        r = RobotConfig.getCurrentRobotInstance() == null? new Robot(hardwareMap) : RobotConfig.getCurrentRobotInstance();
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

        d1Relocalize = driverOp.getGamepadButton(GamepadKeys.Button.OPTIONS);
        d2Relocalize = driver2Op.getGamepadButton(GamepadKeys.Button.OPTIONS);

        shootClose = driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        shootFar = driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);

        lift = driverOp.getGamepadButton(GamepadKeys.Button.TOUCHPAD);

        d1Stop = driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        d2Stop = driver2Op.getGamepadButton(GamepadKeys.Button.DPAD_UP);

        decreaseTurretOffset = driver2Op.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        increaseTurretOffset = driver2Op.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);

        intake.whenPressed(new runIntakeTimed(r, 2000));
        outtake.whenPressed(new runIntakeReverseTimed(r, 2000));
        d1Relocalize.whenPressed(new InstantCommand(() -> r.getD().reloc()));
        shootClose.whenPressed(new magDump(r));
        shootFar.whenPressed(new BOPBOPBOP(r));
        lift.whenPressed(new InstantCommand(() -> r.getL().toggle()));
        d1Stop.whenPressed(new stop(r));
        d2Stop.whenPressed(new stop(r));
        decreaseTurretOffset.whenPressed(new InstantCommand(() -> r.getD().decreaseOffset()));
        increaseTurretOffset.whenPressed(new InstantCommand(() -> r.getD().increaseOffset()));

        schedule(new InstantCommand(() -> r.getD().follower.startTeleOpDrive()));
        schedule(new InstantCommand(()->r.getL().reverseAll()));
        schedule(new InstantCommand(()->r.getS().reverseAll()));
        schedule(new RunCommand(() -> r.setShooterValues()));
        schedule(new RunCommand(()->r.getD().updateTargetAndRelocPose()));
        schedule(new RunCommand(()->r.getD().joystickOffset(driver2Op.getLeftX())));
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
        telemetry.addData("target pose", r.getD().getTargetPose());

        telemetry.update();
        super.run();
    }
}
