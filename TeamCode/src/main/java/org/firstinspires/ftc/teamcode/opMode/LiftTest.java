package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

@Config
@TeleOp(name = "LiftTest")
public class LiftTest extends CommandOpMode {
    GamepadEx driverOp;
    Button lift;
    Button zero;
    private Robot r;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        super.reset();
        r = new Robot(hardwareMap);
        register(r.getL());
        driverOp = new GamepadEx(gamepad1);

        lift = driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        zero = driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);

        lift.whenPressed(new InstantCommand(() -> r.getL().toggle()));
    }

    @Override
    public void run() {
        super.run();
    }

}
