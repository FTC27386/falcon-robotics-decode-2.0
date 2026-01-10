package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class BOPBOPBOP extends SequentialCommandGroup {
    private final Robot r;


    public BOPBOPBOP(Robot r) {
        this.r = r;
        addRequirements(r.getI(), r.getS());
        addCommands(
                new oneShot(r),
                new oneShot(r),
                new oneShot(r)
        );
    }
}
