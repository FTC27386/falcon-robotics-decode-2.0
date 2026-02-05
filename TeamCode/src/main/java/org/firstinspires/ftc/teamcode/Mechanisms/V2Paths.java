package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class V2Paths {
    public static Pose startingPose = new Pose(64.1883, 132.5852, Math.toRadians(180));
    public PathChain closeAutoStartPath;
    public PathChain intakeSecondRowPath;
    public PathChain returnToShootPath;
    public PathChain openGatePath;
    public PathChain intakeFromGatePath;
    public PathChain returnFromGateToShootPath;
    public PathChain intakeFirstRowPath;
    public PathChain returnFromIntakeFirstToShootPath;
    public PathChain intakeThirdRowPath;
    public PathChain returnFromThirdRowToShootPath;
    public PathChain parkPath;

    public V2Paths(Follower follower) {
        closeAutoStartPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(0.000, 144.000),

                                new Pose(60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
        intakeSecondRowPath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(60.000, 57.000),
                                new Pose(10.000, 57.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        returnToShootPath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.000, 57.000),
                                new Pose(60.000, 57.000),
                                new Pose(60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();
        openGatePath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(60.000, 60.000),
                                new Pose(14.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();
        intakeFromGatePath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(16.000, 60.000),
                                new Pose(18.000, 52.000),
                                new Pose(10.000, 52.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(140))
                .build();
        returnFromGateToShootPath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.000, 52.000),
                                new Pose(60.000, 55.000),
                                new Pose(60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))

                .build();
        intakeFirstRowPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),

                                new Pose(20.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        returnFromIntakeFirstToShootPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.000, 84.000),

                                new Pose(60.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
        intakeThirdRowPath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(60.000, 36.000),
                                new Pose(20.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        returnFromThirdRowToShootPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.000, 36.000),

                                new Pose(60.000, 84.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
        parkPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),

                                new Pose(20.000, 64.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}