package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

public class oneShot extends SequentialCommandGroup {
    private final Robot r;


    public oneShot(Robot r) {
        this.r = r;
        addRequirements(r.getI(), r.getS());
        addCommands(
                new InstantCommand(() -> r.getI().close()),
                new preShootSequence(r),
                new runIntake(r),
                new pulseGate(r, 125),
                new WaitCommand(200),
                new idleIntake(r),
                new InstantCommand(() -> r.getS().setSpeed(0))
        );
    }
}
