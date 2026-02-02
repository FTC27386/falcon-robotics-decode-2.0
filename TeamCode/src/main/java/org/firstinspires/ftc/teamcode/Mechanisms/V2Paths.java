package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class V2Paths {

    public static Pose startingPose = new Pose(64.1883,132.5852,0);
    public static Pose startingPoseAlternate = new Pose(53.5946, 7.587, Math.toRadians(90));
    public static Pose startingPoseFarZone = new Pose((54),7,Math.toRadians(0));
    public PathChain StartPath;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;

    public void Paths(Follower follower) {
        StartPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(0.000, 144.000),
                                new Pose(72.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(72.000, 60.000),
                                new Pose(20.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.000, 60.000),
                                new Pose(60.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(17.286, 40.047),
                                new Pose(10.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.000, 60.000),
                                new Pose(17.286, 40.047),
                                new Pose(60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),
                                new Pose(19.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.000, 84.000),
                                new Pose(60.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),
                                new Pose(50.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 36.000),
                                new Pose(19.000, 36.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.000, 36.000),
                                new Pose(60.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}