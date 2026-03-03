package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Utility.RobotConfig;

public class V3Paths {
    public static Pose startingPose = new Pose(64.1883, 132.5852, Math.toRadians(180));
    public static Pose alternatePose = new Pose(144-startingPose.getX(), startingPose.getY(), Math.toRadians(0));

    public PathChain Start;
    public PathChain Intake2nd;
    public PathChain Intake2ndReturn;
    public PathChain GateOpen;
    public PathChain GateIntake;
    public PathChain GateReturn;
    public PathChain Intake1st;
    public PathChain Intake1stReturn;
    public PathChain StartIntake3rd;
    public PathChain EndIntake3rd;
    public PathChain ReturnIntake3rd;

    public V3Paths(Follower follower, RobotConfig.ALLIANCE_COLOR color) {
        Start = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(0.000, 144.000),
                                new Pose(52.409, 97.581),
                                new Pose(46.336, 103.101),
                                new Pose(45.000, 58.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Intake2nd = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 58.000),
                                new Pose(10.000, 58.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Intake2ndReturn = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.000, 58.000),
                                new Pose(44.724, 59.276),
                                new Pose(55.000, 84.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        GateOpen = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.000, 84.000),
                                new Pose(50.717, 46.795),
                                new Pose(12.000, 58.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(140))
                .build();

        GateIntake = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 58.000),

                                new Pose(10.000, 52.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(120))
                .build();

        GateReturn = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.000, 52.000),
                                new Pose(39.992, 50.776),
                                new Pose(55.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .setReversed()
                .build();

        Intake1st = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 84.000),
                                new Pose(16.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Intake1stReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 84.000),

                                new Pose(55.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                .build();

        StartIntake3rd = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 84.000),

                                new Pose(55.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))

                .build();

        EndIntake3rd = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 36.000),

                                new Pose(9.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        ReturnIntake3rd = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.000, 36.000),

                                new Pose(52.000, 120.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}