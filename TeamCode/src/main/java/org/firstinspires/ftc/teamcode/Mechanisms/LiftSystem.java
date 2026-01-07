package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

public class LiftSystem extends SubsystemBase {

    DcMotor lift_motor;
    Servo latch;
    PIDController motor_controller;
    double power;
    int currentPos;
    boolean activated = false;
    double lift_target = 0;
    int total_offset = 0;

    public LiftSystem(HardwareMap hmap) {
        motor_controller = new PIDController(RobotConstants.lift_kP, 0, RobotConstants.lift_kD);
        motor_controller.setSetPoint(0);
        lift_motor = hmap.get(DcMotor.class, RobotConstants.lift_motor_name);
        latch = hmap.get(Servo.class, RobotConstants.lift_servo_name);
        latch.setDirection(Servo.Direction.REVERSE);
        lift_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getLiftPose() {
        return currentPos;
    }

    @Override
    public void periodic() {
        if (activated) {
            currentPos = lift_motor.getCurrentPosition();
            power = activated ?
                    motor_controller.calculate(currentPos, lift_target+total_offset) : 0;
            lift_motor.setPower(power + RobotConstants.lift_kF);
        }
    }

    public void setActivated(boolean activated) {
        this.activated = activated;
    }

    public void unlatch() {
        latch.setPosition(RobotConstants.latch_open_pos);
    }

    public void latch() {
        latch.setPosition(RobotConstants.latch_close_pos);
    }

    public void down() {
        lift_target = RobotConstants.top_climb_position;
    }

    public double getPIDResponse() {
        return motor_controller.calculate();
    }

    public void nudgeLift(int offset)
    {
        total_offset+=offset;
    }
    public void resetOffset()
    {
        total_offset = 0;
    }


}
