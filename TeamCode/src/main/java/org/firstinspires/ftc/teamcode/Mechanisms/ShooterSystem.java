package org.firstinspires.ftc.teamcode.Mechanisms;

import static androidx.core.math.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.GATE_CLOSED_POS;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.GATE_OPEN_POS;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.HOOD_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.HOOD_MIN_POSITION;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.TURRET_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.TURRET_MIDDLE_POSITION;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.TURRET_MIN_POSITION;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Utility.IntakeConfig;
import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.Utility.ShooterConfig;
import org.firstinspires.ftc.teamcode.Utility.cachedMotor;

public class ShooterSystem extends SubsystemBase {
    PIDController headingControl;
    PIDController speedControl;
    Servo turret1;
    Servo turret2;
    Servo hood;
    cachedMotor shooter1;
    cachedMotor shooter2;
    Servo gate;
    double gatePosition;
    double rawCalcPower;
    double degreeRead;
    double previousread;
    double powerToSet;
    double turretPosition;
    double hoodPosition;
    double currentSpeed;

    public ShooterSystem(final HardwareMap hMap) {
        shooter1 = new cachedMotor(hMap.get(DcMotorEx.class, RobotConfig.first_shooter_motor_name),0.06);
        shooter2 = new cachedMotor(hMap.get(DcMotorEx.class, RobotConfig.second_shooter_motor_name),0.06);
        shooter1.thisMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.thisMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turret1 = hMap.get(Servo.class, RobotConfig.left_turret_servo_name);
        turret2 = hMap.get(Servo.class, RobotConfig.right_turret_servo_name);
        hood = hMap.get(Servo.class, RobotConfig.hood_servo_name);
        gate = hMap.get(Servo.class, RobotConfig.transfer_servo_name);
        speedControl = new PIDController(RobotConfig.shooter_kP, 0, RobotConfig.shooter_kD);
        headingControl = new PIDController(RobotConfig.turret_kP, 0, RobotConfig.turret_kD);

        hoodPosition = HOOD_MIN_POSITION;
        turretPosition = TURRET_MIDDLE_POSITION;
        gatePosition = GATE_CLOSED_POS;

    }

    @Override
    public void periodic() {
        // Get voltage and compare, hopefully should filter out drops.
        speedControl.setTolerance(RobotConfig.shooter_tolerance);
        speedControl.setPID(RobotConfig.shooter_kP, 0, RobotConfig.shooter_kD);
        headingControl.setPID(RobotConfig.turret_kP, 0, RobotConfig.turret_kD);

        currentSpeed = shooter1.thisMotor.getVelocity();
        rawCalcPower = speedControl.calculate(-currentSpeed);
        powerToSet = rawCalcPower + (RobotConfig.shooter_kS) + (speedControl.getSetPoint() * RobotConfig.shooter_kV);

        shooter1.setPower(powerToSet);
        shooter2.setPower(powerToSet);

        turret1.setPosition(turretPosition);
        turret2.setPosition(turretPosition);
        hoodPosition = clamp(hoodPosition, 0, 0.576);
        hood.setPosition(hoodPosition);
        gate.setPosition(gatePosition);
        previousread = degreeRead;
    }
    public PIDController getSpeedControl() {
        return speedControl;
    }
    public double getFlywheelSignal() {
        return powerToSet;
    }
    public double getCurrentSpeed() {
        return currentSpeed;
    }
    public void setGate(boolean requestOpen) {
        boolean open = requestOpen && readyToFire();
        gatePosition = open
                ? GATE_OPEN_POS
                : GATE_CLOSED_POS;
    }

    public boolean readyToFire() {
        return atFlywheelSpeed() && atTurretPosition();
    }

    public void setFlywheelSpeed(double speed) {
        speed = clamp(speed, ShooterConfig.FLYWHEEL_MIN_SPEED, ShooterConfig.FLYWHEEL_MAX_SPEED);
        speedControl.setSetPoint(speed);
    }

    public boolean atFlywheelSpeed() {
        return speedControl.atSetPoint();
    }

    public void setTurretPosition(double turretPositionAngle) { // this will take an angle and add it to resting position
        double turretTicks = 0.5;
        turretTicks += turretPositionAngle * RobotConfig.turret_conversion_factor_DEGREES;
        turretTicks = clamp(turretTicks, TURRET_MIN_POSITION, TURRET_MAX_POSITION);
        this.turretPosition = turretTicks;
    }

    public boolean atTurretPosition() {
        return headingControl.atSetPoint();
    }

    public void setHoodAngle(double hoodAngle) {
        hoodAngle = clamp(hoodAngle, HOOD_MIN_POSITION, HOOD_MAX_POSITION);
        hoodPosition = hoodAngle;
    }

    public double getTurretPosition() {
        return turretPosition;
    }

    public double getHoodAngle() {
        return hoodPosition;
    }
}