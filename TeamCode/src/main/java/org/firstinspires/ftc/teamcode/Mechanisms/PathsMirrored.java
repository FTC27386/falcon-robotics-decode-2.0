package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PathsMirrored {

    public static Pose startingPose = new Pose(mirror(64.1883),132.5852,Math.toRadians(180));
    public static Pose startingPoseFarZone = new Pose(mirror(12),8.2,Math.toRadians(0));
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
    public static Pose startingPoseAlternate = new Pose(144-53.5946, 7.587, Math.toRadians(90));
    public PathChain farAutoStartPath;
    public PathChain farLeavePath;
    public PathChain openGatePath;
    public PathChain returnFromGatePath;
    public static double mirror(double xVal)
    {
        return 144.0 - xVal;
    }

    public PathsMirrored(Follower follower) {
        openGatePath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(mirror(14.000), 80.000),
                                new Pose(mirror(33.863), 94.949),
                                new Pose(mirror(32.867), 70.548),
                                new Pose(mirror(16), 70.5)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
        returnFromGatePath =
                follower.pathBuilder().addPath(
                                new BezierLine(new Pose(mirror(16),70.5),
                                        new Pose(mirror(59.1),79)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(0))
                        .build();
        closeAutoStartPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mirror(0),144), new Pose(mirror(59.1), 79.0))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        intakeFirstRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(mirror(59.100), 79.000),
                                new Pose(mirror(49.349), 85.797),
                                new Pose(mirror(14.000), 80.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        returnFromTopRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(mirror(14.000), 80.000),
                                new Pose(mirror(48.958), 85.407),
                                new Pose(mirror(59.1), 79.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        prepareIntakeMiddleRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mirror(59.1), 79.000), new Pose(mirror(44.000), 57))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        intakeMiddleRowPath = follower
                // Y IS 56 INSTEAD OF 57
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mirror(44.000), 56), new Pose(mirror(8.000), 57))
                )
                .setTangentHeadingInterpolation()
                .build();

        returnFromMiddleRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(mirror(8.000), 57),
                                new Pose(mirror(47.300), 60.000),
                                new Pose(mirror(59.100), 79.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        prepareIntakeBottomRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mirror(59.100), 79.000), new Pose(mirror(44.000), 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        intakeBottomRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mirror(44.000), 36.000), new Pose(mirror(8.000), 36.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        returnFromBottomRowPath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(mirror(8.000), 36.000),
                                new Pose(mirror(52.845), 36.228),
                                new Pose(mirror(36.201), 55.993),
                                new Pose(mirror(59.100), 79.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goToGatePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mirror(59.100), 79.000), new Pose(mirror(29.960), 71.181))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
        park= follower
                //DIFFERENT
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(12, 12), new Pose(mirror(39), 39))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        prepareIntakeHPZonePath = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(mirror(56.666), 8.500), new Pose(mirror(12.000), 8.200)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        intakeHPZonePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mirror(12.000), 8.200), new Pose(mirror(12.500), 22.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        returnFromHPZonePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mirror(12.500), 22.000), new Pose(mirror(56.666), 8.500))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        farAutoStartPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mirror(60), 8.5), new Pose(mirror(56.666),8.5))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        farLeavePath = follower
                .pathBuilder()
                //DIFFERENCE
                .addPath(new BezierLine(new Pose(mirror(56.666), 8.500), new Pose(105.6, 8.6)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }
}