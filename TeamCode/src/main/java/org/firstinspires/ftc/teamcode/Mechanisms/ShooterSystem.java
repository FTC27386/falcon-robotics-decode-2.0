package org.firstinspires.ftc.teamcode.Mechanisms;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;
import org.firstinspires.ftc.teamcode.Utility.cachedMotor;

public class ShooterSystem extends SubsystemBase {
    boolean isWoundUp = false;
    boolean initialized;
    PIDController headingControl, speedControl;
    Servo hood;
    cachedMotor shooter1;
    cachedMotor shooter2;
    DcMotorEx turret;
    double rawCalcPower,
    dynamic_offset,
    turretRawPower,
    turretAdjPower,
    angleOffset = 0,
            axonRead = 0,
            degreeRead,
            deltaRead,
            previousread,
            rotations,
            degreestravelled,
            turretDeg,
            error,
            signal,
            powerToSet,
            turretSetPoint,
    turretCurrentPosition, //degrees
            hoodPosition = 0,
            currentSpeed,
            nominalVoltage = 12.00; //voltage at which the shooter was tuned
            //note: we will be plugging in the throughbore to the turret motor encoder. Call it as a normal encoder.

    public ShooterSystem(final HardwareMap hMap) {
        shooter1 = new cachedMotor(
                hMap.get(DcMotorEx.class, RobotConstants.first_shooter_motor_name),0.06);
        shooter2 = new cachedMotor(
                hMap.get(DcMotorEx.class, RobotConstants.second_shooter_motor_name),0.06);
        shooter1.thisMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.thisMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turret = hMap.get(DcMotorEx.class, RobotConstants.turret_motor_name);
        hood = hMap.get(Servo.class, RobotConstants.hood_servo_name);
        speedControl = new PIDController(RobotConstants.shooter_kP, 0, RobotConstants.shooter_kD);
        headingControl = new PIDController(RobotConstants.turret_kP, 0, RobotConstants.turret_kD);
        initialized = false;
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
    public void nudgeOffset(double increment){
        angleOffset += increment;
    }
    public void zeroOffset(){
        angleOffset = 0;
    }
    public void setInitialized(boolean init) {initialized = init;}
    public void forcePower(double power)
    {
        turret.setPower(power);
    }

    @Override
    public void periodic() {
        // Get voltage and compare, hopefully should filter out drops.
        speedControl.setTolerance(RobotConstants.shooter_tolerance);
        speedControl.setPID(RobotConstants.shooter_kP, 0, RobotConstants.shooter_kD);
        headingControl.setPID(RobotConstants.turret_kP, 0, RobotConstants.turret_kD);


        currentSpeed = shooter1.thisMotor.getVelocity();
        turretCurrentPosition = turret.getCurrentPosition() * RobotConstants.encoder_tick_to_degree_conversion - dynamic_offset ; //ticks per rev
        rawCalcPower = speedControl.calculate(-currentSpeed);
        powerToSet = rawCalcPower + (RobotConstants.shooter_kS) + (speedControl.getSetPoint() * RobotConstants.shooter_kV);

            shooter1.setPower(powerToSet);
            shooter2.setPower(powerToSet);

        turretRawPower = headingControl.calculate(turretCurrentPosition, turretSetPoint);
        turretAdjPower = turretRawPower+ RobotConstants.turret_kL;

        if(initialized)
        {
            turret.setPower(turretAdjPower);
        }

        hoodPosition = clamp(hoodPosition, 0, 0.576);
        hood.setPosition(hoodPosition);
    }
    public void hardstopZero()
    {
        dynamic_offset = turret.getCurrentPosition() * RobotConstants.encoder_tick_to_degree_conversion; //effectively zeros everything
    }
    public void setSpeed(double speed) {
        speedControl.setSetPoint(speed);
    }

    public boolean atSpeed() {
        return speedControl.atSetPoint();
    }

    public void setTurretSetPoint(double turretPositionAngle) //this will take an angle
    {
        double turretTicks = 0.5;
        turretTicks += (turretPositionAngle+angleOffset) * RobotConstants.turret_conversion_factor_DEGREES;
        turretTicks = clamp(turretTicks, .2, .8);
        this.turretSetPoint = turretTicks;
    }

    public void setHoodAngle(double hoodAngle) {
        hoodPosition = (hoodAngle - 25) * ((double) 266 /40) * ((double) 1 /300);
    }

    public double getTurretSetPoint() {
        return turretSetPoint;
    }

}