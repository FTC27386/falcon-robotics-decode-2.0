package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;


public class preShootSequence extends CommandBase {

    private final Robot r;


    public preShootSequence(Robot r) {
        this.r = r;
    }

    @Override
    public boolean isFinished() {
        return r.getS().readyToFire() && r.getD().shotAllowed();
    }

}
