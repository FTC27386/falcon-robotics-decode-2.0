package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class followPathSlow extends CommandBase {

    Robot r;
    PathChain path;

    public followPathSlow(Robot r, PathChain path) {
        this.r = r;
        this.path = path;
    }

    @Override
    public void initialize() {
        r.getD().follower.followPath(path, .7, true);
    }

    @Override
    public boolean isFinished() {
        return r.getD().follower.atParametricEnd();
    }
}
