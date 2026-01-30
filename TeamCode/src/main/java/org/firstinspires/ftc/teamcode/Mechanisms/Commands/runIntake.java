package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class runIntake extends CommandBase {

    private final Robot robot;

    public runIntake(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.getI().intake();
        addRequirements(robot.getI());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
