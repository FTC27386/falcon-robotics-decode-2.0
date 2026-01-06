package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mechanisms.Paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class goToLiftPose extends SequentialCommandGroup {

    Robot r;


    public goToLiftPose(Robot r, PathChain parkpath) {
        this.r = r;
        addCommands(
                new followPath(r, parkpath)

        );
    }

}
