package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class autoShootSequenceBlue extends CommandBase {

    private final Robot robot;


    public autoShootSequenceBlue(Robot robot) {
        this.robot = robot;
        addRequirements(robot.getS());
    }

    @Override
    public void initialize() {
        robot.setAutoValuesBlue();
    }

    @Override
    public boolean isFinished() {
        return robot.getS().atSpeed();
    }

}