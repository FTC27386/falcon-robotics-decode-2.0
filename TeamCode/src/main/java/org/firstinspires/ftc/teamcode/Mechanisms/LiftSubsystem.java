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
    boolean lift = false;
    public LiftSubsystem(HardwareMap hmap) {
        leftLiftServo = hmap.get(Servo.class, RobotConfig.left_lift_motor_name);
        rightLiftServo = hmap.get(Servo.class, RobotConfig.right_lift_motor_name);
        leftLiftServo.setDirection(Servo.Direction.REVERSE);
        rightLiftServo.setDirection(Servo.Direction.FORWARD);
        leftLiftServo.setPosition(MIN_POS);
        leftLiftServo.setPosition(MIN_POS);
    }
    public void toggle() {
        lift = !lift;
        double liftPosition = lift ? MAX_POS : MIN_POS;
        leftLiftServo.setPosition(liftPosition);
        rightLiftServo.setPosition(liftPosition);
    }
}
