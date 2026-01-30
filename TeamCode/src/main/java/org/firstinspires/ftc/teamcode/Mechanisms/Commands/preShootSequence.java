package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;


public class preShootSequence extends CommandBase {

    private final Robot robot;


    public preShootSequence(Robot robot) {
        this.robot = robot;
        addRequirements(robot.getS());
    }

    @Override
    public void initialize() {
        robot.setShooterValues();
    }

    @Override
    public boolean isFinished() {
        return robot.getS().atFlywheelSpeed();
    }

}
