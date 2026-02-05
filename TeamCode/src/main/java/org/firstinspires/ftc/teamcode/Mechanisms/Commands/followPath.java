package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

public class followPath extends CommandBase {

    Robot r;
    PathChain path;
    boolean holdEnd;

    public followPath(Robot r, PathChain path) {
        this.r = r;
        this.path = path;
        holdEnd = false;
    }
    public followPath(Robot r, PathChain path, boolean holdEnd)
    {
        this.r = r;
        this.path= path;
        this.holdEnd = holdEnd;
    }

    @Override
    public void initialize() {
        r.getD().follower.followPath(path, holdEnd);
    }


    @Override
    public boolean isFinished() {
        return r.getD().follower.atParametricEnd();
    }
}
