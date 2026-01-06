package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

public class intakeSystem extends SubsystemBase {
    DcMotor intakeMotor;
    Servo gate;
    Servo pivot;
    double targetpower,
            gatePosition = RobotConstants.transfer_open_pos,
            pivotPosition = RobotConstants.pivot_down_pos;

    public intakeSystem(HardwareMap hMap) {
        pivot = hMap.get(Servo.class, RobotConstants.intake_servo_name);
        gate = hMap.get(Servo.class, RobotConstants.transfer_servo_name);
        intakeMotor = hMap.get(DcMotor.class, RobotConstants.intake_motor_name);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void close() {
        gatePosition = (RobotConstants.transfer_closed_pos);
    }

    public void open() {
        gatePosition = (RobotConstants.transfer_open_pos);
    }

    public void toggle(boolean x) {
        if (x) {
            open();
        }
        else {
            close();
        }
    }

    public void intake() {
        targetpower = -1;
    }

    public void idleIntake() {
        targetpower = -0.2;
    } // Passively run intake
    public void stopIntake()
    {
        targetpower = 0;
    }
    public void outtake() {
        targetpower = 1;
    }

    @Override
    public void periodic() {
        intakeMotor.setPower(targetpower);
        gate.setPosition(gatePosition);
        pivot.setPosition(pivotPosition);
    }

    public void stow() {
        pivotPosition = (RobotConstants.pivot_up_pos);
    }
    public void deploy() {pivotPosition=(RobotConstants.pivot_down_pos);}
}
