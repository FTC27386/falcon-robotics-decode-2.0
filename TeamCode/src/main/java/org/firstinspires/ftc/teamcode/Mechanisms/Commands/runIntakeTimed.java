package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class runIntakeTimed extends SequentialCommandGroup {

    public runIntakeTimed(Robot robot, int time) {
        addCommands(
                new runIntake(robot),
                new WaitCommand(time),
                new idleIntake(robot)
        );
    }
}
