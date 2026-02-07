package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class toggleShooter extends CommandBase {

    private final Robot r;

    public toggleShooter(Robot r) {
        this.r = r;
        addRequirements(r.getS());
    }

    @Override
    public void initialize() {
        r.getS().toggle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
