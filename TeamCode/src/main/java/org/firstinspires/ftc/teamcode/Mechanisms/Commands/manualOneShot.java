package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class manualOneShot extends SequentialCommandGroup {
    private final Robot r;


    public manualOneShot(Robot r, double speed, double hood) {
        this.r = r;
        addRequirements(r.getI(), r.getS());
        addCommands(
                new InstantCommand(() -> r.getI().close()),
                new manualPreShootSequence(r, speed, hood),
                new runIntake(r),
                new zonelessPulseGate(r, 100),
                new idleIntake(r)
        );
    }
}
