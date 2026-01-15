package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class manualTriple extends SequentialCommandGroup {
    private final Robot r;


    public manualTriple(Robot r, double speed, double hood) {
        this.r = r;
        addRequirements(r.getI(), r.getS());
        addCommands(
                new InstantCommand(() -> r.getI().close()),
                new manualPreShootSequence(r, speed, hood),
                new runIntake(r),
                new zonelessPulseGate(r, 1200),
                new WaitCommand(200),
                new idleIntake(r),
                new InstantCommand(() -> r.getS().setSpeed(0))
        );
    }
}
