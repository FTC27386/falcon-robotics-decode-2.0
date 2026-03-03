package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Utility.RobotConfig;

public class FZPaths {
        public static Pose startingPose = new Pose(49.55, 10,Math.toRadians(180));
        public static Pose alternatePose = new Pose(144-startingPose.getX(), startingPose.getY(),Math.toRadians(0));
        public Pose HPintake = new Pose(15.000, 10);
        public Pose HPintakeSweep = new Pose(15,15.9);
        public Pose shootingA = new Pose(48.000,10);
        public Pose approachPt3rdSpike = new Pose(36,30);
        public Pose approachFacingPt = new Pose(30,36);
        public Pose ctrlPt3rdSpike = new Pose(33.000, 36.000);
        public Pose endPt3rdSpike = new Pose(12, 36);
        public Pose shootingB = new Pose(54,15);
        public Pose blindIntakeEndPt = new Pose(15, 15);
        public Pose leavePt = new Pose (39,15);
        public PathChain intakeHP;
        public PathChain intakeHPreturn;
        public PathChain intake3rdSpikeA;
        public PathChain intake3rdSpikeB;
        public PathChain return3rdSpike;
        public PathChain blindIntake;
        public PathChain blindIntakeReturn;
        public PathChain intakeHPSweepPath;
        public PathChain leave;

        public FZPaths(Follower follower, RobotConfig.ALLIANCE_COLOR color) {

        if(color == RobotConfig.ALLIANCE_COLOR.BLUE) {
            intakeHP = follower.pathBuilder().addPath(
                            new BezierLine(
                                    startingPose,
                                    HPintake
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            intakeHPSweepPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    HPintake,

                                    HPintakeSweep
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();


            intakeHPreturn = follower.pathBuilder().addPath(
                            new BezierLine(
                                    HPintakeSweep,

                                    shootingB
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intake3rdSpikeA = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingB,

                                    approachPt3rdSpike
                            )
                    ).setHeadingInterpolation(HeadingInterpolator.facingPoint(approachFacingPt))
                    .build();

            intake3rdSpikeB = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    approachPt3rdSpike,
                                    ctrlPt3rdSpike,
                                    endPt3rdSpike
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            return3rdSpike = follower.pathBuilder().addPath(
                            new BezierLine(
                                    endPt3rdSpike,

                                    shootingB
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            blindIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingB,

                                    blindIntakeEndPt
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            blindIntakeReturn = follower.pathBuilder().addPath(
                            new BezierLine(
                                   blindIntakeEndPt,

                                    shootingB
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingB,

                                    leavePt
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
        else
        {
            intakeHP = follower.pathBuilder().addPath(
                            new BezierLine(
                                    startingPose.mirror(),

                                    HPintake.mirror()
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
            intakeHPSweepPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    HPintake.mirror(),

                                    HPintakeSweep.mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            intakeHPreturn = follower.pathBuilder().addPath(
                            new BezierLine(
                                    HPintake.mirror(),

                                    shootingB.mirror()
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intake3rdSpikeA = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingB.mirror(),

                                    approachPt3rdSpike.mirror()
                            )
                    ).setHeadingInterpolation(HeadingInterpolator.facingPoint(approachFacingPt.mirror()))
                    .build();

            intake3rdSpikeB = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    approachPt3rdSpike.mirror(),
                                    ctrlPt3rdSpike.mirror(),
                                    endPt3rdSpike.mirror()
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            return3rdSpike = follower.pathBuilder().addPath(
                            new BezierLine(
                                    endPt3rdSpike.mirror(),

                                    shootingB.mirror()
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            blindIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingB.mirror(),

                                    blindIntakeEndPt.mirror()
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            blindIntakeReturn = follower.pathBuilder().addPath(
                            new BezierLine(
                                    blindIntakeEndPt.mirror(),

                                    shootingB.mirror()
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingB.mirror(),

                                    leavePt.mirror()
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
        }

}
