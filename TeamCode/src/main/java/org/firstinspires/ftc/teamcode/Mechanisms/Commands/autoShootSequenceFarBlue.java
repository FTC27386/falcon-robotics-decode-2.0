package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class autoShootSequenceFarBlue extends CommandBase {

    private final Robot robot;


    public autoShootSequenceFarBlue(Robot robot) {
        this.robot = robot;
        addRequirements(robot.getS());
    }

    @Override
    public void initialize() {
        robot.setAutoValuesFarZoneBlue();
    }

    @Override
    public boolean isFinished() {
        return robot.getS().atSpeed();
    }

}