package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class V2Paths {
    public static Pose startingPose = new Pose(64.1883, 132.5852, Math.toRadians(180));

    public Pose touchGoalPose = new Pose(0,144);
    public Pose shootingPose = new Pose(60, 84);
    public Pose midSpikePoseA = new Pose(8,52);
    public Pose gateOpenPose = new Pose(16,60);
    public Pose gateIntakePose = new Pose(10,55);
    public Pose topSpikeEndpoint = new Pose(18,82);
    public Pose midSpikeControlPoint = new Pose(60,60);
    public Pose midSpikeReturnControlPoint = new Pose(60,57);
    public Pose openGateControlPoint = new Pose(60,64);
    public Pose intakeGateControlPoint = new Pose(18,52);
    public Pose gateReturnControlPoint = new Pose(60,55);
    public Pose thirdSpikeControlPoint = new Pose(60,36);
    public Pose thirdSpikeEndPose = new Pose(12,36);
    public Pose leavePose = new Pose(20,64);
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
                                touchGoalPose,
                                shootingPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
        intakeSecondRowPath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootingPose,
                                midSpikeControlPoint,
                                midSpikePoseA
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        returnToShootPath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                midSpikePoseA,
                                midSpikeReturnControlPoint,
                                shootingPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();
        openGatePath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootingPose,
                               openGateControlPoint,
                               gateOpenPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();
        intakeFromGatePath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                gateOpenPose,
                                intakeGateControlPoint,
                                gateIntakePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(140))
                .build();
        returnFromGateToShootPath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                gateIntakePose,
                                gateReturnControlPoint,
                                shootingPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))

                .build();
        intakeFirstRowPath = follower.pathBuilder().addPath(
                        new BezierLine(
                               shootingPose,

                                topSpikeEndpoint
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        returnFromIntakeFirstToShootPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                topSpikeEndpoint,

                                shootingPose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
        intakeThirdRowPath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootingPose,
                                thirdSpikeControlPoint,
                               thirdSpikeEndPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        returnFromThirdRowToShootPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                thirdSpikeEndPose,

                               shootingPose
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
        parkPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootingPose,

                                leavePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}