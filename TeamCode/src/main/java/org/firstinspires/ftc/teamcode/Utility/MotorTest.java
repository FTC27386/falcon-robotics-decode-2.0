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
package org.firstinspires.ftc.teamcode.Utility;

import static androidx.core.math.MathUtils.clamp;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.valueOf;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
@TeleOp(name = "MotorTest", group = "Robot")

public class MotorTest extends OpMode {
    DcMotor flywheel1, flywheel2;
    Servo leftTurretServo, rightTurretServo, intake_raiser;
    DcMotor frontLeftDrive,
            frontRightDrive,
            backLeftDrive,
            backRightDrive,
            intake;
    Servo hood,
            blocker,
            pivot;
    private static double x = 0;
    private static double turretPosition = 0.5;
    private static double servoOffset = 0;
    private static double intakePos = 0;
    private static double hoodPos = 0;

    @Override
    public void init() {
        leftTurretServo = hardwareMap.get(Servo.class, RobotConfig.left_turret_servo_name);
        rightTurretServo = hardwareMap.get(Servo.class, RobotConfig.right_turret_servo_name);
        blocker = hardwareMap.get(Servo.class, RobotConfig.transfer_servo_name);
        hood = hardwareMap.get(Servo.class, RobotConfig.hood_servo_name);
        intake_raiser = hardwareMap.get(Servo.class, RobotConfig.intake_servo_name);

        leftTurretServo.setDirection(Servo.Direction.FORWARD);
        rightTurretServo.setDirection(Servo.Direction.FORWARD);
        blocker.setDirection(Servo.Direction.FORWARD);
        hood.setDirection(Servo.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, RobotConfig.intake_motor_name);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1 = hardwareMap.get(DcMotor.class, RobotConfig.first_shooter_motor_name);
        flywheel2 = hardwareMap.get(DcMotor.class, RobotConfig.second_shooter_motor_name);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setDirection(DcMotor.Direction.FORWARD);
        flywheel1.setZeroPowerBehavior(FLOAT); //Makes the flywheel1 not turn itself off
        flywheel2.setZeroPowerBehavior(FLOAT);

        frontLeftDrive = hardwareMap.get(DcMotor.class, RobotConfig.left_front_drive_motor_name);
        frontRightDrive = hardwareMap.get(DcMotor.class, RobotConfig.right_front_drive_motor_name);
        backLeftDrive = hardwareMap.get(DcMotor.class, RobotConfig.left_back_drive_motor_name);
        backRightDrive = hardwareMap.get(DcMotor.class, RobotConfig.right_back_drive_motor_name);

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) x+= 0.01;
        if (gamepad1.dpadDownWasPressed()) x-= 0.01;
        x = clamp(x, 0, 1);
        hood.setPosition(x);
        intake_raiser.setPosition(intakePos);
        leftTurretServo.setPosition(turretPosition);
        rightTurretServo.setPosition(turretPosition + servoOffset);
        telemetry.addData("x", x);
    }
}
