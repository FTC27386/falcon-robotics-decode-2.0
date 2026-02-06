package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;

@Configurable
public class Constants {

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-116 / 25.4)
            .strafePodX(-91.751 / 25.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .yawScalar(1.0)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(24.5 * .454)
            .centripetalScaling(0.0004)
            .forwardZeroPowerAcceleration(-33.800)
            .lateralZeroPowerAcceleration(-63.8)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.4,0,0,0.0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.002, 0, .012, 0.012))
            .headingPIDFCoefficients(new PIDFCoefficients(2.7, 0.0, .1, 0 ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(.05,0,0.02,0.012))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(3.0,0,0.0004,0.6,0.008))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.013,0,0.0002,0.04,0.012));




    public static MecanumConstants driveConstants = new MecanumConstants()
            .rightFrontMotorName(RobotConfig.right_front_drive_motor_name)
            .leftFrontMotorName(RobotConfig.left_front_drive_motor_name)
            .rightRearMotorName(RobotConfig.right_back_drive_motor_name)
            .leftRearMotorName(RobotConfig.left_back_drive_motor_name)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(80.170481)
            .yVelocity(64.797);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 50, .9, .1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
