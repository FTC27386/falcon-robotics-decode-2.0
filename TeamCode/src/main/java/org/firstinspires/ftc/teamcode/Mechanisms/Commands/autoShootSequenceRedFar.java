package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class autoShootSequenceRedFar extends CommandBase {

    private final Robot robot;


    public autoShootSequenceRedFar(Robot robot) {
        this.robot = robot;
        addRequirements(robot.getS());
    }

    @Override
    public void initialize() {
        robot.setAutoValuesFarZoneRed();
    }

    @Override
    public boolean isFinished() {
        return robot.getS().atSpeed();
    }

}