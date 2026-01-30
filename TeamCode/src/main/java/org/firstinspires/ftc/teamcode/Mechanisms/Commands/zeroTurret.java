package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class zeroTurret extends SequentialCommandGroup {

    public zeroTurret(Robot r)
    {
       addCommands(new InstantCommand(()->r.getS().setInitialized(false)),
               new InstantCommand(()->r.getS().forcePower(.5)),
               new WaitCommand(500),
               new InstantCommand(()->r.getS().zeroOffset()),
               new InstantCommand(()->r.getS().forcePower(0)),
               new InstantCommand(()->r.getS().setInitialized(true)));
    }
}
