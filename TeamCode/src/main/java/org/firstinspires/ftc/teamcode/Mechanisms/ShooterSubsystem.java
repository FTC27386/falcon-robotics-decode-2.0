package org.firstinspires.ftc.teamcode.Mechanisms;

import static androidx.core.math.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.HOOD_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.HOOD_MIN_POSITION;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.TURRET_CONVERSION_FACTOR_RADIANS;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.TURRET_MAX_POW;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private double flywheelSpeed;
    private boolean resetTurret;
    ElapsedTime turretResetTimer = new ElapsedTime();

    public ShooterSubsystem(final HardwareMap hMap) {
        shooter1 = new cachedMotor(hMap.get(DcMotorEx.class, RobotConfig.first_shooter_motor_name),0.06);
        shooter2 = new cachedMotor(hMap.get(DcMotorEx.class, RobotConfig.second_shooter_motor_name),0.06);
        shooter1.thisMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shooter2.thisMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooter1.thisMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.thisMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter1.thisMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.thisMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        turret = hMap.get(DcMotorEx.class, RobotConfig.turret_motor_name);
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();


        hood = hMap.get(Servo.class, RobotConfig.hood_servo_name);
        hood.setDirection(Servo.Direction.FORWARD);

        flywheelPIDController = new PIDController(ShooterConfig.shooter_kP, 0, ShooterConfig.shooter_kD);
        turretPIDController = new PIDController(ShooterConfig.turret_kP, 0, ShooterConfig.turret_kD);

        hoodPosition = HOOD_MIN_POSITION;
        resetTurret();
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

        // FORWARD IN RADIANS IS PI/2 RADIANS
        double currentTurretPosition = UtilMethods.angleWrapRad(turret.getCurrentPosition() * TURRET_CONVERSION_FACTOR_RADIANS);
        double error = UtilMethods.angleWrapRad(turretPIDController.getSetPoint() - currentTurretPosition);
        double turretPower = turretPIDController.calculate(error, 0);
        turretPower = clamp(turretPower, -TURRET_MAX_POW, TURRET_MAX_POW);

        if (resetTurret) {
            if (turretResetTimer.milliseconds() <= 2000) {
                turret.setPower(0.1);
            }
            else {
                turret.setPower(0);
                resetTurret = false;
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        else {
            turret.setPower(turretPower);
        }

        // Encoder ticks increase linearly with turret rotation
        // Encoder = pi/2 rad -> turret forward
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
        flywheelPIDController.setSetPoint(speed);
    }

    public boolean readyToFire() {
        return atFlywheelSpeed() && atTurretPosition();
    }

    public boolean atFlywheelSpeed() {
        return flywheelPIDController.atSetPoint();
    }

    public void setTargetTurretAngle(double targetTurretAngle) { // this will set an angle in radians
        targetTurretAngle = UtilMethods.angleWrapRad(targetTurretAngle);
        turretPIDController.setSetPoint(targetTurretAngle);
    }
    public void resetTurret() {
        resetTurret = true;
        turretResetTimer.reset();
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
