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

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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

@TeleOp(name = "InterpLUT Aim", group = "Robot")

public class TurretTest extends OpMode {
    Pose currentpose;
    double correctionAngle;
    public static InterpLUT lut;
    public static double offsetRadians;
    public static double FLYWHEEL_SPEED;
    public static double targetX = 0,
            targetY = 0,
            yawMultiplier = 1;
    // This declares the four motors needed
    DcMotor flywheel1, flywheel2;
    GoBildaPinpointDriver localizer;
    Servo leftTurretServo, rightTurretServo;
    DcMotor frontLeftDrive,
            frontRightDrive,
            backLeftDrive,
            backRightDrive,
            intake;
    Servo hood,
            blockerServo,
            pivotServo;
    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;
    double field_adjustment_angle,
            trueX,
    trueY,
            HOOD_ANGLE,
            MAX_ANGLE,
            MIN_ANGLE,
            pinpointX,
            distanceX,
            distanceVector,
            pinpointY,
            distanceY,
            robot_relative_angle,
            odo_turretservo_angle;

    boolean block = true;
    boolean pivot = false;
    Follower follower;

    @Override
    public void init() {
        FLYWHEEL_SPEED = 1;
        HOOD_ANGLE = 0.5;
        MAX_ANGLE = 1;
        MIN_ANGLE = 0;

        leftTurretServo = hardwareMap.get(Servo.class, RobotConfig.left_turret_servo_name);
        rightTurretServo = hardwareMap.get(Servo.class, RobotConfig.right_turret_servo_name);
        blockerServo = hardwareMap.get(Servo.class, "blocker");

        leftTurretServo.setDirection(Servo.Direction.FORWARD);
        rightTurretServo.setDirection(Servo.Direction.FORWARD);

        blockerServo.setDirection(Servo.Direction.FORWARD);


        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1 = hardwareMap.get(DcMotor.class, RobotConfig.first_shooter_motor_name);
        flywheel2 = hardwareMap.get(DcMotor.class, RobotConfig.second_shooter_motor_name);
        hood = hardwareMap.get(Servo.class, "hood");

        intake.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setDirection(DcMotor.Direction.FORWARD);
        flywheel1.setZeroPowerBehavior(FLOAT); //Makes the flywheel1 not turn itself off
        flywheel2.setZeroPowerBehavior(FLOAT);

        follower = Constants.createFollower(hardwareMap); //creates a follower w/ bottom left for the pose, pointing straight up. Same as in auto
        follower.setStartingPose(new Pose(8,8,Math.toRadians(90)));
        follower.update();
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update(); // updates follower
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        currentpose = follower.getPose(); //gets current pose just once
        trueX = currentpose.getX();
        trueY = currentpose.getY();

        distanceX = targetX - trueX;
        distanceY = targetY - trueY; //possibly needs flipping based on quadrant
        field_adjustment_angle = (90 - Math.toDegrees(Math.atan2(distanceY,distanceX)));

        //first conversion
        //odo_turretservo_angle = 1-((currentpose.getHeading() + Math.PI/2) + Math.toRadians(313.25)/Math.toRadians(626.52));
       // odo_turretservo_angle +=(Math.toRadians(field_adjustment_angle))/Math.toRadians(626);
        odo_turretservo_angle = 0.5;
        odo_turretservo_angle += Math.toDegrees(currentpose.getHeading() - Math.PI/2) * (0.0015962441314554); // simplified conversion factor. Maybe casting?
        odo_turretservo_angle += field_adjustment_angle * (0.0015962441314554);

        telemetry.addLine("Left trigger is shooter");
        telemetry.addLine("Dpad up/down controls hood angle");
        telemetry.addData("Heading", Math.toDegrees(currentpose.getHeading()));
        telemetry.addData("goal field angle", field_adjustment_angle);
        telemetry.addData("x to goal", distanceX);
        telemetry.addData("y to goal", distanceY);
        telemetry.addData("x", trueX);
        telemetry.addData("y",trueY);
        telemetry.addData("correction servo ticks", Math.toDegrees(currentpose.getHeading() - Math.PI/2) * (0.0015962441314554));

        if (gamepad1.optionsWasPressed()) follower.setPose(new Pose(-10,0, Math.toRadians(90)));
        //if (gamepad1.aWasPressed()) imu.resetYaw();
        if (gamepad1.dpadUpWasPressed()) HOOD_ANGLE += 0.01;
        if (gamepad1.dpadDownWasPressed()) HOOD_ANGLE -= 0.01;

        // Clamp both values between MIN and MAX.
        HOOD_ANGLE = clamp(HOOD_ANGLE, MIN_ANGLE, MAX_ANGLE);
        if (gamepad1.left_trigger > 0) turret(FLYWHEEL_SPEED, HOOD_ANGLE, Math.abs(odo_turretservo_angle));
        else turret(0, HOOD_ANGLE, Math.abs(odo_turretservo_angle));
        if (gamepad1.shareWasPressed()) lut.add(distanceVector, HOOD_ANGLE);
        intake.setPower(gamepad1.right_trigger > 0 ? 1 : 0);
        if (gamepad1.rightBumperWasPressed()) block = !block;
        if (gamepad1.leftBumperWasPressed()) pivot = !pivot;

        if (block) blockerServo.setPosition(0.5);
        else blockerServo.setPosition(0.25);

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

    private void turret(double speed, double angle, double turretAngle) {
        flywheel1.setPower(speed);
        flywheel2.setPower(speed);
        hood.setPosition(angle);
        leftTurretServo.setPosition(turretAngle);
        rightTurretServo.setPosition(turretAngle);
        telemetry.addData("Hood Angle", "Angle %5.2f", hood.getPosition());
        telemetry.addData("Turret Angle", rightTurretServo.getPosition());
        telemetry.addData("turretleft", leftTurretServo.getPosition());
        telemetry.addData("odo_correction_ticks",odo_turretservo_angle);
        telemetry.addData("field_correction_ticks", (-Math.toRadians(field_adjustment_angle))/Math.toRadians(626));
    }
}
