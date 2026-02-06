package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

public class V2Paths {
    public static Pose startingPose = new Pose(64.1883, 132.5852, Math.toRadians(180));

    public Pose touchGoalPose = new Pose(0,144);
    public Pose shootingPose = new Pose(56, 84);

    public Pose midSpikePose = new Pose(10,60);
    public Pose midSpikeControlPoint = new Pose(shootingPose.getX(), midSpikePose.getY());
    public Pose midSpikeReturnControlPoint = new Pose(shootingPose.getX(), midSpikePose.getY());

    public Pose gateOpenPose = new Pose(12,58);
    public Pose openGateControlPoint = new Pose(shootingPose.getX(), gateOpenPose.getY());
    public Pose gateIntakePose = new Pose(10,53);
    public Pose gateIntakeControlPoint = new Pose(gateOpenPose.getX(), gateIntakePose.getY());
    public Pose gateReturnControlPoint = new Pose(shootingPose.getX(), gateIntakePose.getY());

    public Pose topSpikePose = new Pose(10,80);

    public Pose bottomSpikePose = new Pose(10,36);
    public Pose bottomSpikeControlPoint = new Pose(shootingPose.getX(), bottomSpikePose.getY());

    public Pose leavePose = new Pose(50,64);

    public PathChain startPath;
    public PathChain intakeSecondRowPath;
    public PathChain secondRowReturnToShootPath;
    public PathChain openGatePath;
    public PathChain intakeFromGatePath;
    public PathChain returnFromGateToShootPath;
    public PathChain startIntakeFirstRowPath;
    public PathChain returnFromIntakeFirstRowToShootPath;
    public PathChain startIntakeThirdRowPath;
    public PathChain returnFromThirdRowToShootPath;
    public PathChain parkPath;

    PathConstraints preciseConstraints = new PathConstraints(0.998,0.002,0.003,0.02,50,.90,30,.1);

    public V2Paths(Follower follower) {
        startPath = follower.pathBuilder().addPath(
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
                                midSpikePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        secondRowReturnToShootPath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                midSpikePose,
                                midSpikeReturnControlPoint,
                                shootingPose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
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
                                gateIntakeControlPoint,
                                gateIntakePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(130))
                .build();
        returnFromGateToShootPath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                gateIntakePose,
                                gateReturnControlPoint,
                                shootingPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .build();
        startIntakeFirstRowPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootingPose,
                                topSpikePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        returnFromIntakeFirstRowToShootPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                topSpikePose,
                                shootingPose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        startIntakeThirdRowPath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootingPose,
                                bottomSpikeControlPoint,
                                bottomSpikePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        returnFromThirdRowToShootPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                bottomSpikePose,
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