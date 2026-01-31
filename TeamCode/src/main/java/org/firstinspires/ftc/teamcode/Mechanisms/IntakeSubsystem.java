package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utility.RobotConfig;

public class IntakeSubsystem extends SubsystemBase {
    DcMotor intakeMotor;
    double targetPower;

    public IntakeSubsystem(HardwareMap hMap) {
        intakeMotor = hMap.get(DcMotor.class, RobotConfig.intake_motor_name);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        intakeMotor.setPower(targetPower);
    }

    public void intake() {
        targetPower = -1;
    }
    public void outtake() {
        targetPower = 1;
    }
    public void idleIntake() {
        targetPower = -0.2;
    } // Passively run intake
    public void stopIntake()
    {
        targetPower = 0;
    }

}
