package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class runIntake extends CommandBase {

    private final Robot r;

    public runIntake(Robot r) {
        this.r = r;
        addRequirements(r.getI());
    }

    @Override
    public void initialize() {
        r.getI().intake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
