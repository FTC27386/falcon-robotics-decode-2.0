package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class manualShot extends SequentialCommandGroup {
    private final Robot r;


    public manualShot(Robot r) {
        this.r = r;
        addRequirements(r.getG());
        addCommands(
                new InstantCommand(() -> r.getG().close()),
                new runIntake(r),
                new pulseGateOverride(r, 1200),
                new stopIntake(r)
        );
    }
}
