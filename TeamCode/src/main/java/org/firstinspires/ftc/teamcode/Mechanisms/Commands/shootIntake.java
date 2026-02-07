package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class shootIntake extends CommandBase {

    private final Robot r;
    boolean inCloseZone;

    public shootIntake(Robot r, boolean inCloseZone) {
        this.inCloseZone = inCloseZone;
        this.r = r;
        addRequirements(r.getI());
    }

    @Override
    public void initialize() {
        r.getI().zonedIntake(inCloseZone);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
