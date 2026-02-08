/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.temp;

import static androidx.core.math.MathUtils.clamp;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utility.UtilMethods;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */


@Config
@Disabled
@TeleOp(name = "InterpLUT Aim Analog", group = "Robot")


public class InterpLUTAimAnalog extends OpMode {
    public static double flywheel_kP = -0.02, flywheel_kD = 0.000000002, flywheelkF = 0.25;
    public static double kP = 0.012;
    public static double kD = 0;
    public static double kF = 0;
    public static double kL = 0.05;

    private AnalogInput turretEnc;
    public static double turretOffset;
    public static double targetX = 0,
            targetY = 144,
            yawMultiplier = 1;
    // This declares the motors needed
    DcMotorEx flywheel1, flywheel2;
    GoBildaPinpointDriver localizer;
    CRServo leftTurretServo, rightTurretServo;
    DcMotor intake;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    Servo hood;
    Servo blocker;
    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;
    double field_adjustment_angle;
    double trueX;
    double trueY;
    public static double HOOD_ANGLE;
    double MAX_ANGLE;
    double MIN_ANGLE;
    double pinpointX;
    double distanceX;
    double turretDeg;
    double distanceVector;
    double pinpointY;
    double distanceY;
    double error;
    double previousRead;
    double signal;
    double axonRead;
    double degreeRead;
    double deltaRead;
    double odo_turretservo_angle;
    double degreestravelled = 0,flywheelsignal;
    int rotations=0;
    Pose2D pose;
    public static Pose2D bottom_right_pose = new Pose2D(DistanceUnit.INCH, 136, 8, AngleUnit.DEGREES, 90);
    PIDController turretPDFL;
    PIDController flywheel_PDFL;
    public static double flywheel_target = 270;
    public static double flywheel_current;

    boolean block = true;
    ElapsedTime shoot1 = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    public static AprilTagProcessor aprilTag;
    public static VisionPortal visionPortal;
    public static void stateAprilTag(boolean state) {
        visionPortal.setProcessorEnabled(aprilTag, state);
    }
    public InterpLUT lut1 = new InterpLUT();
    public InterpLUT lut2 = new InterpLUT();
    public static double[][] lutArray = new double[5][2];
    public static int lutNum;
    public static double lut1MIN;
    public static double lut2MIN;
    public static double lut1MAX;
    public static double lut2MAX;
    public static int obelisk;
    public static boolean interpLUTActive = true;

    @Override
    public void init() {
        // Speed 225
        lut1MIN = 69.674;
        lut1.add(69.674,0.017);
        lut1.add(81.521,0.029);
        lut1.add(84.143, 0.036);
        lut1MAX = 84.143;

        // Speed 270
        lut2MIN = 82.260;
        lut2.add(82.260,0.084);
        lut2.add(94.016, 0.080);
        lut2.add(116.262,0.068);
        lut2MAX = 116.262;

        lut1.createLUT();
        lut2.createLUT();

        lutNum = 0;
        obelisk = 0;
        /*
        try {
            initAprilTag();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
         */
        turretOffset = 0;
        flywheel_PDFL = new PIDController(flywheel_kP, 0, flywheel_kD);
        turretPDFL = new PIDController(kP, 0, kD);
        flywheel_PDFL.setTolerance(20);

        turretEnc = hardwareMap.get(AnalogInput.class, "turret_encoder");
        localizer = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        localizer.setOffsets(-116, -91.751, DistanceUnit.MM);
        localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        localizer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        localizer.resetPosAndIMU();
        localizer.setPosition(bottom_right_pose);
        HOOD_ANGLE = 0;
        MAX_ANGLE = 1;
        MIN_ANGLE = 0;

        leftTurretServo = hardwareMap.get(CRServo.class, "left_turret_servo");
        rightTurretServo = hardwareMap.get(CRServo.class, "right_turret_servo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel_top");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel_bottom");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        hood = hardwareMap.get(Servo.class, "hood");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setDirection(DcMotor.Direction.FORWARD);
        flywheel2.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setZeroPowerBehavior(FLOAT); //Makes the flywheel1 not turn itself off
        flywheel2.setZeroPowerBehavior(FLOAT);
        intake.setZeroPowerBehavior(BRAKE);

        //imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        // imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        //getAprilTag();
        //encoder test
        flywheel_PDFL.setPID(flywheel_kP,0,flywheel_kD);
        turretPDFL.setPID(kP, 0, kD);
        axonRead = turretEnc.getVoltage();
        degreeRead = axonRead * (360/3.3);
        deltaRead = degreeRead - previousRead;
        if (deltaRead > 180)
        {
            rotations -= 1;
        }
        if (deltaRead < -180)
        {
            rotations += 1;
        }
        degreestravelled = rotations*360.0 + degreeRead;
        turretDeg = degreestravelled*(5.0)*(60/170.0);
        error = UtilMethods.AngleDifference(odo_turretservo_angle, turretDeg) ;
        if(gamepad1.right_trigger > 0)
        {
            flywheel_current = flywheel1.getVelocity(AngleUnit.DEGREES);
            flywheelsignal = flywheel_PDFL.calculate(flywheel_current, flywheel_target);
            flywheelsignal += Math.signum(flywheelsignal) * flywheelkF;
        }
        else
        {
            flywheelsignal = 0;
        }

        signal = turretPDFL.calculate(-error, 0);
        signal += Math.signum(signal) * kL;

        localizer.update();
        pose = localizer.getPosition();
        pinpointX = pose.getX(DistanceUnit.INCH); //VARIABLES USE STANDARD CARTESIAN AXES!!!
        trueY = pinpointX;
        pinpointY = pose.getY(DistanceUnit.INCH);
        trueX = -pinpointY;
        distanceX = targetX - trueX;
        distanceY = targetY - trueY;
        field_adjustment_angle = (90 - Math.toDegrees(Math.atan2(distanceY,distanceX)));
       // odo_turretservo_angle = 1-(localizer.getHeading(AngleUnit.RADIANS)+Math.toRadians(313))/Math.toRadians(626);
        //odo_turretservo_angle +=(Math.toRadians(field_adjustment_angle))/Math.toRadians(626);
       // odo_turretservo_angle *= 360;
        odo_turretservo_angle = -pose.getHeading(AngleUnit.DEGREES) - field_adjustment_angle + turretOffset;

        distanceVector = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
        //distanceVector = Math.atan2(distanceX, distanceY);
        //odo_turretservo_angle = gamepad1.right_trigger;
        //field_relative_angle = Math.atan2(distanceX, distanceY); //Inverted here because X is actually the vertical axis
        //robot_relative_angle = ((Math.PI/2 - field_relative_angle) - ((yawMultiplier * localizer.getHeading(AngleUnit.RADIANS))));
        //odo_turretservo_angle = 0.5 -(-Math.toDegrees(robot_relative_angle) * ((double) 1 / 355) * ((double) 170 / 60) * ((double) 1 / 5));
        //radians of robot-relative angle * conv. to degrees * conv. to servo ticks * GR2 * GR1

        telemetry.addLine("Right trigger is shoot");
        telemetry.addLine("Left trigger is intake in");
        telemetry.addLine("Left bumper is intake out");
        telemetry.addLine("Dpad up/down controls hood angle");
        telemetry.addData("Heading", localizer.getHeading(AngleUnit.DEGREES));
        telemetry.addData("goal field angle", field_adjustment_angle);
        telemetry.addData("odo ang",odo_turretservo_angle);
        telemetry.addData("x to goal", distanceX);
        telemetry.addData("y to goal", distanceY);
        telemetry.addData("PINPOINT Y (horiz axis)", pinpointY);
        telemetry.addData("PINPOINT X (vert)", pinpointX);
        telemetry.addData("Hood Angle", HOOD_ANGLE);
        telemetry.addData("Obelisk", obelisk);
        telemetry.addData("Distance Vector", distanceVector);
        /*
        telemetry.addData("Lut #", lutNum);
        for(int i = 0; i < lutArray.length; i++) {
            telemetry.addData("InterpLUT Value " + i, String.format("%.3f, %.3f", lutArray[i][0], lutArray[i][1]));
        }
        */
        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
        // Set LUT values
        if (gamepad1.shareWasPressed() && lutNum <= lutArray.length) {
            lutArray[lutNum][0] = distanceVector;
            lutArray[lutNum][1] = HOOD_ANGLE;
        }
        else {
            telemetry.addLine("No more values.");
        }
         */
        if (gamepad1.shareWasPressed()) interpLUTActive = !interpLUTActive;

        if (gamepad1.optionsWasPressed()) localizer.resetPosAndIMU();
        if (gamepad1.dpadUpWasPressed()) HOOD_ANGLE += 0.001;
        if (gamepad1.dpadDownWasPressed()) HOOD_ANGLE -= 0.001;
        if (gamepad1.dpadLeftWasPressed()) lutNum -= 1;
        if (gamepad1.dpadRightWasPressed()) lutNum += 1;
        lutNum = clamp(lutNum, 0, lutArray.length);
        HOOD_ANGLE = clamp(HOOD_ANGLE, MIN_ANGLE, MAX_ANGLE);

        if (gamepad1.aWasPressed()) {
            block = !block;
        }

        /*
        if (gamepad1.aWasPressed()) {
            shoot.reset();
            block = false;
        }
        if (shoot.milliseconds() > 125) {
            block = true;
        }
         */

        /*
        if (gamepad1.triangleWasPressed()) {
            block = !block;
        }
         */

        blocker.setPosition(block ? 0.5 : 0.25);
        // Clamp both values between MIN and MAX.

        // Shooter control
        telemetry.addLine("REGRESSION ACTIVE");
        if (distanceVector > lut1MIN && distanceVector < lut2MIN) {
            telemetry.addLine("Zone 1");
            HOOD_ANGLE = 0.000114507 * Math.pow(distanceVector, 2)
                       - 0.0163 * distanceVector
                       + 0.596814;
            flywheel_target = 225;
        } else if (distanceVector > lut2MIN && distanceVector < lut2MAX) {
            telemetry.addLine("Zone 2");
            HOOD_ANGLE = -0.00000585763 * Math.pow(distanceVector, 2)
                       + 0.000692307 * distanceVector
                       + 0.0666877;
            flywheel_target = 270;
        } else {
            flywheel_target = 300;
        }
        turret(flywheelsignal, HOOD_ANGLE, (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) ? 0 : signal);

        // Intake control
        if (gamepad1.left_trigger > 0) {
            intake.setPower(1);
        }
        else if (gamepad1.left_bumper) {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }
        previousRead = degreeRead;
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    private void turret(double speed, double angle, double turretPower) {
        flywheel1.setPower(speed);
        flywheel2.setPower(speed);
        leftTurretServo.setPower(turretPower);
        rightTurretServo.setPower(turretPower);
        hood.setPosition(angle);
        telemetry.addData("power set", flywheel1.getPower());
        telemetry.addData("degrees Travelled", degreestravelled);
        telemetry.addData("current servo pos", degreeRead);
        telemetry.addData("pose_rotation", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("error", error);
        telemetry.addData("odo_correction_ticks",odo_turretservo_angle);
        telemetry.addData("field_correction_ticks", (-Math.toRadians(field_adjustment_angle))/Math.toRadians(626));
        telemetry.addData("rotations", rotations);
        telemetry.addData("turretDeg", turretDeg);
        telemetry.addData("flywheel_power", flywheelsignal);
        telemetry.addData("angular velocity", flywheel1.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("flywheel_current", flywheel_current);
        telemetry.addData("flywheel_target", flywheel_target);

    }

    private void getAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                if (detection.id == 21 || detection.id == 22 || detection.id == 23) {
                    obelisk = detection.id;
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
    }
    private void initAprilTag() throws InterruptedException {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure((long)5, TimeUnit.MILLISECONDS);
        sleep(20);

    }   // end method initAprilTag()
}
