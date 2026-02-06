package org.firstinspires.ftc.teamcode.opMode;

import static androidx.core.math.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Mechanisms.Commands.goToLiftPose;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.defaultDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.followPath;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.magDump;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.runIntakeTimed;
import org.firstinspires.ftc.teamcode.Mechanisms.Commands.idleIntake;
import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

import java.util.function.Supplier;

@Config
@TeleOp(name = "Manual Stepping Thru Auto")
public class AutoStepThru extends CommandOpMode {
    int index = 0;
    Button intake;
    Button relocalize;
    Button shoot;
    Button changeTarget;
    GamepadEx driverOp;
    Button climb;
    Button dpadup;
    Button dpaddown;
    public static double hood_pos = 0;
    public static double flywheel_speed = -1500;
    private Robot r;
    Paths paths;
    public Command[] AutoCommands;

    @Override
    public void initialize() {
        super.reset();
        r = new Robot(hardwareMap);
        register(r.getS(), r.getG(), r.getD(), r.getI(), r.getL());
        driverOp = new GamepadEx(gamepad1);
        Supplier<Double> leftX = driverOp::getLeftX;
        Supplier<Double> leftY = driverOp::getLeftY;
        Supplier<Double> rightX = driverOp::getRightX;
        r.getD().setDefaultCommand(new defaultDrive(r, leftY, leftX, rightX));

        intake = driverOp.getGamepadButton(GamepadKeys.Button.SQUARE);
        relocalize = driverOp.getGamepadButton(GamepadKeys.Button.OPTIONS);
        changeTarget = driverOp.getGamepadButton(GamepadKeys.Button.SHARE);
        shoot = driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        climb = driverOp.getGamepadButton(GamepadKeys.Button.TOUCHPAD);
        paths = new Paths(r.getD().follower);

        climb.whenPressed(new goToLiftPose(r, paths.park));
        intake.whenPressed(new runIntakeTimed(r, 2000));
        relocalize.whenPressed(new InstantCommand(() -> r.getD().reloc()));

        schedule(new InstantCommand(() -> r.getD().follower.startTeleOpDrive()));
        schedule(new RunCommand(() -> r.getS().setTurretAngle(r.getD().getTurret())));
        shoot.whenPressed(new magDump(r));
        AutoCommands = new Command[]{
                new InstantCommand(() -> r.setShooterValues()),
                new InstantCommand(() -> r.getG().close()),
                new followPath(r, paths.closeAutoStartPath),
                new magDump(r),
                new runIntake(r),
                new InstantCommand(() -> r.setShooterValues()),
                new followPath( r, paths.intakeFirstRowPath), //intake 1st line
                new idleIntake(r),
                new followPath(r, paths.returnFromTopRowPath), //return to shoot point
                new magDump(r),
                new followPath(r, paths.prepareIntakeMiddleRowPath),
                new runIntake(r),
                new InstantCommand(() -> r.setShooterValues()),
                new followPath(r, paths.intakeMiddleRowPath),
                new idleIntake(r),
                new followPath(r, paths.returnFromMiddleRowPath),
                new magDump(r),
                new followPath(r, paths.prepareIntakeBottomRowPath),
                new runIntake(r),
                new InstantCommand(() -> r.setShooterValues()),
                new followPath(r, paths.intakeBottomRowPath),
                new idleIntake(r),
                new followPath(r, paths.returnFromBottomRowPath),
                new magDump(r),
                new followPath(r, paths.goToGatePath)
        };


    }
    @Override
    public void run() {

        if (gamepad1.dpadUpWasPressed())
        {
                    schedule(AutoCommands[index]);
        }
        if(gamepad1.dpadLeftWasPressed())
        {
            index -= 1;
        }
        if(gamepad1.dpadRightWasPressed())
        {
            index +=1;
        }
        index = clamp(index, 0, AutoCommands.length);


        telemetry.addData("x:", r.getD().getCurrentPose().getX());
        telemetry.addData("y:", r.getD().getCurrentPose().getY());
        telemetry.addData("heading", r.getD().getCurrentPose().getHeading());
        telemetry.addData("turret", r.getD().getTurret());
        telemetry.addData("flywheel target velocity", r.getS().getFlywheelTarget());
        telemetry.addData("flywheel speed", r.getS().getFlywheelSpeed());
        telemetry.addData("Hood", r.getD().getHood());
        telemetry.addData("flywheel response", r.getS().getFlywheelPower());
        telemetry.addData("turret ticks", r.getS().getTurretTarget());
        telemetry.addData("Distance", r.getD().getDist());
        telemetry.addData("current step", index);

        telemetry.update();
        super.run();
    }

}
