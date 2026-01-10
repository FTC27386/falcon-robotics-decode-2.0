package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    public static Pose startingPose = new Pose(64.1883,132.5852,0);
    public static Pose startingPoseAlternate = new Pose(53.5946, 7.587, Math.toRadians(90));
    public static Pose startingPoseFarZone = new Pose((54),7,Math.toRadians(0));
    public PathChain closeAutoStartPath;
    public PathChain intakeFirstRowPath;
    public PathChain returnFromTopRowPath;
    public PathChain prepareIntakeMiddleRowPath;
    public PathChain intakeMiddleRowPath;
    public PathChain returnFromMiddleRowPath;
    public PathChain prepareIntakeBottomRowPath;
    public PathChain intakeBottomRowPath;
    public PathChain returnFromBottomRowPath;
    public PathChain goToGatePath;
    public PathChain park;
    public PathChain prepareIntakeHPZonePath;
    public PathChain intakeHPZonePath;
    public PathChain returnFromHPZonePath;
    public PathChain farAutoStartPath;
    public PathChain farLeavePath;
    public PathChain openGatePath;
    public PathChain returnFromGatePath;





    public Paths(Follower follower) {
        openGatePath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.000, 57.000),
                                new Pose(37.349, 53.865),
                                new Pose(33.863, 73.536),
                                new Pose(15.000, 72.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        returnFromGatePath =
                follower.pathBuilder().addPath(
                        new BezierLine(new Pose(15,72),
                                new Pose(59.1,79)
                                )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();
        closeAutoStartPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(0,144), new Pose(59.1, 79.0))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        intakeFirstRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.100, 79.000),
                                new Pose(49.349, 85.797),
                                new Pose(14.000, 80.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        returnFromTopRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(14.000, 80.000),
                                new Pose(48.958, 85.407),
                                new Pose(59.1, 79.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        prepareIntakeMiddleRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.1, 79.000), new Pose(44.000, 57))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        intakeMiddleRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.000, 57), new Pose(8.000, 57))
                )
                .setTangentHeadingInterpolation()
                .build();

        returnFromMiddleRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.000, 57),
                                new Pose(47.300, 60.000),
                                new Pose(59.100, 79.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        prepareIntakeBottomRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.100, 79.000), new Pose(44.000, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        intakeBottomRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.000, 36.000), new Pose(8.000, 36.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        returnFromBottomRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.000, 36.000),
                                new Pose(52.845, 36.228),
                                new Pose(36.201, 55.993),
                                new Pose(59.100, 79.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToGatePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.100, 79.000), new Pose(29.960, 71.181))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
        park= follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132, 12), new Pose(39, 39))
                )
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();

        prepareIntakeHPZonePath = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(56.666, 8.500), new Pose(12.000, 8.200)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        intakeHPZonePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(12.000, 8.200), new Pose(12.500, 22.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        returnFromHPZonePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(12.500, 22.000), new Pose(56.666, 8.500))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        farAutoStartPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60, 8.5), new Pose(56.666,8.5))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        farLeavePath = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(56.666, 8.500), new Pose(38.4, 8.6)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
}