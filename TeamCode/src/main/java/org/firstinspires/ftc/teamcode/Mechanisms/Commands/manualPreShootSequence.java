package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;


public class manualPreShootSequence extends CommandBase {

    private final Robot robot;
    private double speed;
    private double hood;


    public manualPreShootSequence(Robot robot, double speed, double hood) {
        this.robot = robot;
        this.speed = speed;
        this.hood = hood;
        addRequirements(robot.getS());
    }

    @Override
    public void initialize() {
        robot.setManualShooterValues(speed, hood);
    }

    @Override
    public boolean isFinished() {
        return robot.getS().atSpeed();
    }

}
