package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class pulseGateOverride extends SequentialCommandGroup {
    Robot r;
    int delay;

    public pulseGateOverride(Robot r, int delay) {
        this.r = r;
        this.delay = delay;
        addRequirements(r.getS());
        addCommands(
                new InstantCommand(() -> r.getS().setGate(true)),
                new WaitCommand(delay),
                new InstantCommand(() -> r.getS().setGate(false)),
                new WaitCommand(200)
        );
    }
}
