package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class magDump extends SequentialCommandGroup {
    private final Robot r;


    public magDump(Robot r) {
        this.r = r;
        addRequirements(r.getG());
        addCommands(
                new preShootSequence(r),
                new runIntake(r),
                new pulseGate(r, 1200),
                new idleIntake(r)
        );
    }
}
