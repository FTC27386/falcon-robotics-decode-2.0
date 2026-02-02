package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class oneShot extends SequentialCommandGroup {
    private final Robot r;


    public oneShot(Robot r) {
        this.r = r;
        addRequirements(r.getG());
        addCommands(
                new runIntake(r),
                new pulseGate(r, 100),
                new idleIntake(r)
        );
    }
}
