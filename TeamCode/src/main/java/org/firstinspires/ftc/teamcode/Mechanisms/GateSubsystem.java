package org.firstinspires.ftc.teamcode.Mechanisms;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.GATE_CLOSED_POS;
import static org.firstinspires.ftc.teamcode.Utility.ShooterConfig.GATE_OPEN_POS;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utility.RobotConfig;

public class GateSubsystem extends SubsystemBase {
    private Servo gate;
    private double gatePosition;

    public GateSubsystem(final HardwareMap hMap) {
        gate = hMap.get(Servo.class, RobotConfig.transfer_servo_name);
        gate.setDirection(Servo.Direction.FORWARD);
        gatePosition = GATE_CLOSED_POS;

    }

    @Override
    public void periodic() {
        gate.setPosition(gatePosition);
    }

    public void open() {
        gatePosition = GATE_OPEN_POS;
    }

    public void close() {
        gatePosition = GATE_CLOSED_POS;
    }

}