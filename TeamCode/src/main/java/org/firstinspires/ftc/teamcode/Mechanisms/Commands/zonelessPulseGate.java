package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class zonelessPulseGate extends SequentialCommandGroup {
    Robot r;
    int delay;

    public zonelessPulseGate(Robot r, int delay) {
        this.r = r;
        this.delay = delay;
        addRequirements(r.getI());
        addCommands(
                new WaitCommand(delay),
                new InstantCommand(() -> r.getI().close()),
                new WaitCommand(200)
        );
    }
}
