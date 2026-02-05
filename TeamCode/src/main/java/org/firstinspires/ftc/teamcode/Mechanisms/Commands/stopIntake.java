package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class stopIntake extends CommandBase {

    private final Robot robot;

    public stopIntake(Robot robot) {
        this.robot = robot;
        addRequirements(robot.getI());
    }

    @Override
    public void initialize() {
        robot.getI().stopIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
