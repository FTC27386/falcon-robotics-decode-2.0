package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class magDump extends SequentialCommandGroup {
    private final Robot r;


    public magDump(Robot r) {
        this.r = r;

        addRequirements(r.getG());
        addCommands(
                new runIntake(r),
                new pulseGate(r, 200),
                new pulseGate(r, 200),
                new pulseGate(r, 200),
                new stopIntake(r),
                new InstantCommand(() -> r.getG().close()));
    }
}
