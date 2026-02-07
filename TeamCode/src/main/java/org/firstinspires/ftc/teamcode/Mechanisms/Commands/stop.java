package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class stop extends SequentialCommandGroup {

    private final Robot r;

    public stop(Robot r) {
        this.r = r;
        addRequirements(r.getS(), r.getI());
        addCommands(
                new toggleShooter(r),
                new stopIntake(r)
        );
    }
}
