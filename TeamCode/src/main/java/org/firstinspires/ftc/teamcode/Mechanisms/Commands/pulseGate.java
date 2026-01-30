package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class pulseGate extends SequentialCommandGroup {
    Robot r;
    int delay;

    public pulseGate(Robot r, int delay) {
        this.r = r;
        this.delay = delay;
        addRequirements(r.getS());
        addCommands(
                new InstantCommand(() -> r.getS().setGate(r.getD().shotAllowed())),
                new WaitCommand(delay),
                new InstantCommand(() -> r.getS().setGate(false)),
                new WaitCommand(200)
        );
    }
}
