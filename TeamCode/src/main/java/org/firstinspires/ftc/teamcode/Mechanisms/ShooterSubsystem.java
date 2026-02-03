package org.firstinspires.ftc.teamcode.Mechanisms;

import static androidx.core.math.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.HOOD_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.HOOD_MIN_POSITION;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.TURRET_CONVERSION_FACTOR_RADIANS;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.TURRET_MAX_POW;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.TURRET_OFFSET;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Utility.RobotConfig;
import org.firstinspires.ftc.teamcode.Utility.ShooterConfig;
import org.firstinspires.ftc.teamcode.Utility.UtilMethods;
import org.firstinspires.ftc.teamcode.Utility.cachedMotor;

public class ShooterSubsystem extends SubsystemBase {
    private PIDController turretPIDController;
    private PIDController flywheelPIDController;
    private DcMotorEx turret;
    private Servo hood;
    private cachedMotor shooterTop;
    private cachedMotor shooterBottom;
    private double flywheelPower;
    private double hoodPosition;
    private double error;
    private double flywheelSpeed;
    private double turretAngle;
    private double turretPower;

    public ShooterSubsystem(final HardwareMap hMap) {
        shooterTop = new cachedMotor(hMap.get(DcMotorEx.class, RobotConfig.first_shooter_motor_name),0.06);
        shooterBottom = new cachedMotor(hMap.get(DcMotorEx.class, RobotConfig.second_shooter_motor_name),0.06);
        shooterTop.thisMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterBottom.thisMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shooterTop.thisMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterBottom.thisMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterTop.thisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterBottom.thisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterTop.thisMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom.thisMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret = hMap.get(DcMotorEx.class, RobotConfig.turret_motor_name);
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = hMap.get(Servo.class, RobotConfig.hood_servo_name);
        hood.setDirection(Servo.Direction.FORWARD);

        flywheelPIDController = new PIDController(ShooterConfig.shooter_kP, 0, ShooterConfig.shooter_kD);
        turretPIDController = new PIDController(ShooterConfig.turret_kP, 0, ShooterConfig.turret_kD);

        hoodPosition = HOOD_MIN_POSITION;
    }

    @Override
    public void periodic() {
        flywheelPIDController.setTolerance(ShooterConfig.SHOOTER_TOLERANCE);
        flywheelPIDController.setPID(ShooterConfig.shooter_kP, 0, ShooterConfig.shooter_kD);

        turretPIDController.setTolerance(ShooterConfig.TURRET_TOLERANCE_RADIANS);
        turretPIDController.setPID(ShooterConfig.turret_kP, 0, ShooterConfig.turret_kD);

        flywheelSpeed = shooterTop.thisMotor.getVelocity();
        double rawCalcPower = flywheelPIDController.calculate(-flywheelSpeed);
        flywheelPower = rawCalcPower + (ShooterConfig.shooter_kS) + (flywheelPIDController.getSetPoint() * ShooterConfig.shooter_kV);

        shooterTop.setPower(flywheelPower);
        shooterBottom.setPower(flywheelPower);

        // FORWARD IN RADIANS IS PI/2 RADIANS
        turretAngle = (turret.getCurrentPosition() * TURRET_CONVERSION_FACTOR_RADIANS + TURRET_OFFSET);
        error = turretPIDController.getSetPoint() - turretAngle;
        turretPower = turretPIDController.calculate(error, 0);
        turretPower = clamp(turretPower, -TURRET_MAX_POW, TURRET_MAX_POW);

        turret.setPower(turretPower);

        // Encoder ticks increase linearly with turret rotation
        // Encoder = pi/2 rad -> turret forward
        // Positive ticks correspond to positive rotation direction (CCW)

        hoodPosition = clamp(hoodPosition, HOOD_MIN_POSITION, HOOD_MAX_POSITION);
        hood.setPosition(hoodPosition);
    }
    public double getFlywheelSpeed() {
        return flywheelSpeed;
    }
    public double getFlywheelPower() {
        return flywheelPower;
    }
    public void setFlywheelSpeed(double speed) {
        flywheelPIDController.setSetPoint(speed);
    }
    public double getFlywheelTarget() {
        return flywheelPIDController.getSetPoint();
    }
    public boolean atFlywheelSpeed() {
        return flywheelPIDController.atSetPoint();
    }
    public boolean readyToFire() {
        return atFlywheelSpeed();
    }
    public double getError()
    {
        return  error;
    }
    public double getTurretAngle() {
        return turretAngle;
    }
    public double getTurretPower() {
        return turretPower;
    }
    public void setTurretAngle(double turretAngle) { // this will set a target angle in radians
        turretAngle = (turretAngle);
        turretPIDController.setSetPoint(turretAngle);
    }

    public double getTurretTarget() {
        return flywheelPIDController.getSetPoint();
    }
    public boolean atTurretAngle() {
        return turretPIDController.atSetPoint();
    }

    public void setHoodAngle(double hoodAngle) {
        hoodAngle = clamp(hoodAngle, HOOD_MIN_POSITION, HOOD_MAX_POSITION);
        hoodPosition = hoodAngle;
    }

    public double getHoodAngle() {
        return hoodPosition;
    }
}
