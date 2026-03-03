package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public  class difffzpaths {
    public PathChain Path1;
    public PathChain Path8;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    public difffzpaths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(49.000, 10.000),

                                new Pose(15.000, 10.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.000, 10.000),

                                new Pose(15.000, 15.900)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setReversed()
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.000, 15.900),

                                new Pose(54.000, 15.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.000, 15.000),

                                new Pose(36.000, 30.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(36.000, 30.000),
                                new Pose(33.000, 36.000),
                                new Pose(12.000, 36.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 36.000),

                                new Pose(54.000, 15.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.000, 15.000),

                                new Pose(15.000, 15.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.000, 15.000),

                                new Pose(54.000, 15.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}











