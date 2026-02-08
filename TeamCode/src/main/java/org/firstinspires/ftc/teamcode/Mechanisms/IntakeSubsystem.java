package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.Utility.IntakeConfig.FAR_ZONE_INTAKE;
import static org.firstinspires.ftc.teamcode.Utility.IntakeConfig.IDLE_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.Utility.IntakeConfig.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.Utility.IntakeConfig.OUTTAKE_POWER;
import static org.firstinspires.ftc.teamcode.Utility.IntakeConfig.STOP_INTAKE_POWER;

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
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void periodic() {
        intakeMotor.setPower(targetPower);
    }

    public void intake() {
        targetPower = INTAKE_POWER;
    }
    public void outtake() {
        targetPower = OUTTAKE_POWER;
    }
    public void idleIntake() {
        targetPower = IDLE_INTAKE_POWER;
    } // Passively run intake
    public void farZoneIntake() { targetPower = FAR_ZONE_INTAKE; }
    public void zonedIntake(boolean inCloseZone) {
        if (inCloseZone) {
            intake();
        }
        else {
            farZoneIntake();
        }
    }
    public void stopIntake() {
        targetPower = STOP_INTAKE_POWER;
    }

}
