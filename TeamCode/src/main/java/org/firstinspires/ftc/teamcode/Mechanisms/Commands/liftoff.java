package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class liftoff extends SequentialCommandGroup {
    Robot r;
    public liftoff(Robot r)
    {
        this.r = r;
        addCommands(
                new InstantCommand(()->r.getS().setFlywheelSpeed(0)),
                new InstantCommand(()->r.getI().stopIntake()),
                new WaitCommand(1000),
        new InstantCommand(()->r.getL().setActivated(true)),
        new InstantCommand(()->r.getL().down())
        );
    }
}
