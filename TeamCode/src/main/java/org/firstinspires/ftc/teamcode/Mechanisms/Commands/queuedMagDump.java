package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class queuedMagDump extends SequentialCommandGroup {
    private final Robot r;


    public queuedMagDump(Robot r) {
        this.r = r;
        addRequirements(r.getG());
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                        new InstantCommand(()->r.getG().close()),
                        new runIntake(r),
                        new pulseGate(r,200),
                        new pulseGate(r,200),
                        new pulseGate(r,200)),
                        new WaitCommand(0),
                        ()-> r.getD().shotAllowed()
                ));

    }
}
