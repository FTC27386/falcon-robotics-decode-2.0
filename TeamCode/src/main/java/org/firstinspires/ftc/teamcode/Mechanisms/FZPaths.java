package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Utility.RobotConfig;

public class FZPaths {

        public PathChain intakeHP;
        public PathChain intakeHPreturn;
        public PathChain intake3rdSpikeA;
        public PathChain intake3rdSpikeB;
        public PathChain return3rdSpike;
        public PathChain blindIntake;
        public PathChain blindIntakeReturn;
        public PathChain leave;
        public static Pose initPose = new Pose(49.55, 10,Math.toRadians(180)),
        HPintake = new Pose(15.000, 10),
        shootingA = new Pose(48.000,10),
        approachPt3rdSpike = new Pose(36,30),
        approachFacingPt = new Pose(30,36),
        ctrlPt3rdSpike = new Pose(33.000, 36.000),
        endPt3rdSpike = new Pose(12, 36),
        shootingB = new Pose(54,15),
        blindIntakeEndPt = new Pose(15, 15),
        leavePt = new Pose (39,15);



        public FZPaths(Follower follower, RobotConfig.ALLIANCE_COLOR color) {

        if(color == RobotConfig.ALLIANCE_COLOR.BLUE) {
            intakeHP = follower.pathBuilder().addPath(
                            new BezierLine(
                                    initPose,

                                    HPintake
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            intakeHPreturn = follower.pathBuilder().addPath(
                            new BezierLine(
                                    HPintake,

                                    shootingA
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intake3rdSpikeA = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingA,

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
                                    initPose.mirror(),

                                    HPintake.mirror()
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            intakeHPreturn = follower.pathBuilder().addPath(
                            new BezierLine(
                                    HPintake.mirror(),

                                    shootingA.mirror()
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intake3rdSpikeA = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingA.mirror(),

                                    approachPt3rdSpike.mirror()
                            )
                    ).setHeadingInterpolation(HeadingInterpolator.facingPoint(approachFacingPt))
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
