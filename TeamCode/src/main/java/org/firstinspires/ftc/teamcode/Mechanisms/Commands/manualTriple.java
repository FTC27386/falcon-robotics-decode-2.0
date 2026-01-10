package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class manualTriple extends SequentialCommandGroup {
    private final Robot r;


    public manualTriple(Robot r, double speed, double hood) {
        this.r = r;
        addRequirements(r.getI(), r.getS());
        addCommands(
                new manualOneShot(r, speed, hood),
                new manualOneShot(r, speed, hood),
                new manualOneShot(r, speed, hood)
        );
    }
}
