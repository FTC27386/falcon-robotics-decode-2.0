package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.Utility.RobotConfig;

public class V2Paths {
    public static Pose startingPose = new Pose(64.1883, 132.5852, Math.toRadians(180));
    public static Pose alternatePose = new Pose(144-startingPose.getX(), startingPose.getY(), Math.toRadians(0));

    public Pose touchGoalPose = new Pose(0,144);
    public Pose shootingPose = new Pose(54, 84);

    public Pose midSpikePose = new Pose(10,57);
    public Pose midSpikeControlPoint = new Pose(shootingPose.getX(), midSpikePose.getY());
    public Pose midSpikeReturnControlPoint = new Pose(shootingPose.getX(), midSpikePose.getY());

    public Pose gateOpenPose = new Pose(12,58);
    public Pose openGateControlPoint = new Pose(shootingPose.getX(), gateOpenPose.getY());
    public Pose gateIntakePose = new Pose(10,53);
    public Pose gateIntakeControlPoint = new Pose(gateOpenPose.getX(), gateIntakePose.getY());
    public Pose gateReturnControlPoint = new Pose(shootingPose.getX(), gateIntakePose.getY());

    public Pose topSpikePose = new Pose(10,80);
    public Pose bottomSpikePoseA = new Pose(shootingPose.getX(), 34);
    public Pose bottomSpikePoseB = new Pose(10, bottomSpikePoseA.getY());

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
    public PathChain endIntakeThirdRowPath;
    public PathChain returnFromThirdRowToShootPath;
    public PathChain parkPath;

    PathConstraints ramToWall = new PathConstraints(.250,350,.90,.1);

    public V2Paths(Follower follower, RobotConfig.ALLIANCE_COLOR color) {

        if(color == RobotConfig.ALLIANCE_COLOR.BLUE) {
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
                .setConstraints(ramToWall)
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
                .setConstraints(ramToWall)
                .build();
        intakeFromGatePath = follower.pathBuilder().addPath(
                        new BezierCurve(
                                gateOpenPose,
                                gateIntakeControlPoint,
                                gateIntakePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(130))
                .setConstraints(ramToWall)
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
                .setConstraints(ramToWall)
                .build();
        returnFromIntakeFirstRowToShootPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                topSpikePose,
                                shootingPose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        startIntakeThirdRowPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootingPose,
                                bottomSpikePoseA
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        endIntakeThirdRowPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                bottomSpikePoseA,
                                bottomSpikePoseB
                        )
        ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setConstraints(ramToWall)
                .build();
        returnFromThirdRowToShootPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                bottomSpikePoseB,
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
                .build();}
        else {
            startPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    touchGoalPose.mirror(),
                                    shootingPose.mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();
            intakeSecondRowPath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootingPose.mirror(),
                                    midSpikeControlPoint.mirror(),
                                    midSpikePose.mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setConstraints(ramToWall)
                    .build();
            secondRowReturnToShootPath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    midSpikePose.mirror(),
                                    midSpikeReturnControlPoint.mirror(),
                                    shootingPose.mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            openGatePath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootingPose.mirror(),
                                    openGateControlPoint.mirror(),
                                    gateOpenPose.mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .setConstraints(ramToWall)
                    .build();
            intakeFromGatePath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    gateOpenPose.mirror(),
                                    gateIntakeControlPoint.mirror(),
                                    gateIntakePose.mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(50))
                    .setConstraints(ramToWall)
                    .build();
            returnFromGateToShootPath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    gateIntakePose.mirror(),
                                    gateReturnControlPoint.mirror(),
                                    shootingPose.mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();
            startIntakeFirstRowPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingPose.mirror(),
                                    topSpikePose.mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setConstraints(ramToWall)
                    .build();
            endIntakeThirdRowPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingPose.mirror(),
                                    topSpikePose.mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setConstraints(ramToWall)
                    .build();
            returnFromIntakeFirstRowToShootPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    topSpikePose.mirror(),
                                    shootingPose.mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            startIntakeThirdRowPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingPose.mirror(),
                                    bottomSpikePoseA.mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            endIntakeThirdRowPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    bottomSpikePoseA.mirror(),
                                    bottomSpikePoseB.mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setConstraints(ramToWall)
                    .build();
            returnFromThirdRowToShootPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    bottomSpikePoseB.mirror(),
                                    shootingPose.mirror()
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            parkPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootingPose.mirror(),
                                    leavePose.mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
    }
}