package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class BOPBOPBOP extends SequentialCommandGroup {
    private final Robot r;


    public BOPBOPBOP(Robot r) {
        this.r = r;

        addRequirements(r.getG());
        addCommands(
                new runIntake(r),
                new pulseGate(r, 200),
                new InstantCommand(() -> r.getG().close()),
                new WaitCommand(350),

                new pulseGate(r, 200),
                new InstantCommand(() -> r.getG().close()),
                new WaitCommand(350),

                new pulseGate(r, 200),
                new InstantCommand(() -> r.getG().close()),

                new idleIntake(r));
    }
}
