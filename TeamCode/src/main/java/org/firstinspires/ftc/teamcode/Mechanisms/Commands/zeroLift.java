package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class zeroLift extends SequentialCommandGroup {
    private final Robot r;
    public zeroLift(Robot r) {
        this.r = r;
        addRequirements(r.getL());
    }

    @Override
    public void initialize() {
        r.getL().zero();
    }

    @Override
    public boolean isFinished() { return true; }
}
