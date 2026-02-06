package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.Utility.LiftConfig.MAX_POS;
import static org.firstinspires.ftc.teamcode.Utility.LiftConfig.MIN_POS;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utility.RobotConfig;

public class LiftSubsystem extends SubsystemBase {

    Servo leftLiftServo;
    Servo rightLiftServo;
    public LiftSubsystem(HardwareMap hmap) {
        leftLiftServo = hmap.get(Servo.class, RobotConfig.left_lift_motor_name);
        rightLiftServo = hmap.get(Servo.class, RobotConfig.right_lift_motor_name);
        leftLiftServo.setDirection(Servo.Direction.REVERSE);
        rightLiftServo.setDirection(Servo.Direction.FORWARD);
        deactivate();
    }
    public void deactivate() {
        leftLiftServo.setPosition(MIN_POS);
        rightLiftServo.setPosition(MIN_POS);
    }
    public void activate() {
        leftLiftServo.setPosition(MAX_POS);
        rightLiftServo.setPosition(MAX_POS);
    }
}
