package org.firstinspires.ftc.teamcode.Mechanisms;

import static androidx.core.math.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.HOOD_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.HOOD_MIN_POSITION;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.TURRET_CONVERSION_FACTOR_RADIANS;

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
    private cachedMotor shooter1;
    private cachedMotor shooter2;
    private double flywheelPower;
    private double targetTurretAngle;
    private double hoodPosition;
    public double flywheelSpeed;

    public ShooterSubsystem(final HardwareMap hMap) {
        shooter1 = new cachedMotor(hMap.get(DcMotorEx.class, RobotConfig.first_shooter_motor_name),0.06);
        shooter2 = new cachedMotor(hMap.get(DcMotorEx.class, RobotConfig.second_shooter_motor_name),0.06);
        shooter1.thisMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shooter2.thisMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooter1.thisMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.thisMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turret = hMap.get(DcMotorEx.class, RobotConfig.turret_motor_name);
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hood = hMap.get(Servo.class, RobotConfig.hood_servo_name);
        hood.setDirection(Servo.Direction.FORWARD);

        flywheelPIDController = new PIDController(ShooterConfig.shooter_kP, 0, ShooterConfig.shooter_kD);
        turretPIDController = new PIDController(ShooterConfig.turret_kP, 0, ShooterConfig.turret_kD);

        hoodPosition = HOOD_MIN_POSITION;

    }

    @Override
    public void periodic() {
        flywheelPIDController.setTolerance(ShooterConfig.shooter_tolerance);
        flywheelPIDController.setPID(ShooterConfig.shooter_kP, 0, ShooterConfig.shooter_kD);

        turretPIDController.setTolerance(ShooterConfig.TURRET_TOLERANCE_RADIANS);
        turretPIDController.setPID(ShooterConfig.turret_kP, 0, ShooterConfig.turret_kD);

        flywheelSpeed = shooter1.thisMotor.getVelocity();
        double rawCalcPower = flywheelPIDController.calculate(-flywheelSpeed);
        flywheelPower = rawCalcPower + (ShooterConfig.shooter_kS) + (flywheelPIDController.getSetPoint() * ShooterConfig.shooter_kV);

        shooter1.setPower(flywheelPower);
        shooter2.setPower(flywheelPower);

        // QUICK N' DIRTY
        double targetAngle = targetTurretAngle; // radians, ideally already wrapped [-pi, pi]
        targetAngle = UtilMethods.angleWrapRad(targetAngle);

        double currentAngle = turret.getCurrentPosition() * TURRET_CONVERSION_FACTOR_RADIANS;

        double error = UtilMethods.angleWrapRad(targetAngle - currentAngle);

        double turretPower = Math.signum(error) * 0.2;

        turret.setPower(turretPower);

        // FORWARD IN RADIANS IS 0 RADIANS
        /*
        double currentTurretPosition = turret.getCurrentPosition() * TURRET_CONVERSION_FACTOR_RADIANS;
        currentTurretPosition = UtilMethods.angleWrapRad(currentTurretPosition);
        double targetTurretRad = turretPosition;
        double error = UtilMethods.angleWrapRad(targetTurretRad - currentTurretPosition);
        double turretPower = turretPIDController.calculate(error, 0);
        turretPower = clamp(turretPower, -TURRET_MAX_POW, TURRET_MAX_POW);
        turret.setPower(turretPower);
         */

        // Encoder ticks increase linearly with turret rotation
        // Encoder = 0 rad -> turret forward
        // Positive ticks correspond to positive rotation direction (CCW)

        hoodPosition = clamp(hoodPosition, HOOD_MIN_POSITION, HOOD_MAX_POSITION);
        hood.setPosition(hoodPosition);
    }
    public PIDController getFlywheelPIDController() {
        return flywheelPIDController;
    }
    public double getFlywheelSignal() {
        return flywheelPower;
    }
    public double getFlywheelSpeed() {
        return flywheelSpeed;
    }

    public void setFlywheelSpeed(double speed) {
        speed = clamp(speed, ShooterConfig.FLYWHEEL_MIN_SPEED, ShooterConfig.FLYWHEEL_MAX_SPEED);
        flywheelPIDController.setSetPoint(speed);
    }

    public boolean readyToFire() {
        return atFlywheelSpeed() && atTurretPosition();
    }

    public boolean atFlywheelSpeed() {
        return flywheelPIDController.atSetPoint();
    }

    public void resetTurret() {
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setTargetTurretAngle(double targetTurretAngle) { // this will set an angle in radians
        targetTurretAngle = UtilMethods.angleWrapRad(targetTurretAngle);
        turretPIDController.setSetPoint(targetTurretAngle);
    }

    public boolean atTurretPosition() {
        return turretPIDController.atSetPoint();
    }

    public void setHoodAngle(double hoodAngle) {
        hoodAngle = clamp(hoodAngle, HOOD_MIN_POSITION, HOOD_MAX_POSITION);
        hoodPosition = hoodAngle;
    }

    public double getTargetTurretAngle() {
        return targetTurretAngle;
    }

    public double getHoodAngle() {
        return hoodPosition;
    }
}