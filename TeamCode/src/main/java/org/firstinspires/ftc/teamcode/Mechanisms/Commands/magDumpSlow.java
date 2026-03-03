package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class magDumpSlow extends SequentialCommandGroup {
    private final Robot r;


    public magDumpSlow(Robot r) {
        this.r = r;

        addRequirements(r.getG());
        addCommands(
                new runIntake(r),
                new pulseGate(r, 200),
                new idleIntake(r));
    }
}
