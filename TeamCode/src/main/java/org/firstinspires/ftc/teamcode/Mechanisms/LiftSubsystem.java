package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.Utility.LiftConfig.LIFT_POWER;
import static org.firstinspires.ftc.teamcode.Utility.LiftConfig.LIFT_TIME;
import static org.firstinspires.ftc.teamcode.Utility.LiftConfig.PASSIVE_POWER;
import static org.firstinspires.ftc.teamcode.Utility.LiftConfig.ZERO_POWER;
import static org.firstinspires.ftc.teamcode.Utility.LiftConfig.ZERO_TIME;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utility.RobotConfig;

public class LiftSubsystem extends SubsystemBase {

    CRServo leftLiftServo;
    CRServo rightLiftServo;
    double power;
    boolean runningTimed = false;
    boolean lift = false;
    double timedPower = -0.1;
    double timedMs = 1000;
    ElapsedTime timer = new ElapsedTime();

    public LiftSubsystem(HardwareMap hmap) {
        leftLiftServo = hmap.get(CRServo.class, RobotConfig.left_lift_motor_name);
        rightLiftServo = hmap.get(CRServo.class, RobotConfig.right_lift_motor_name);
        leftLiftServo.setDirection(CRServo.Direction.REVERSE);
        rightLiftServo.setDirection(CRServo.Direction.FORWARD);
        zero();
    }

    @Override
    public void periodic() {
        if (runningTimed) {
            if (timer.milliseconds() >= timedMs) {
                if (lift) {
                    power = PASSIVE_POWER;
                }
                else {
                    power = 0;
                }
                runningTimed = false;
            }
            else {
                power = timedPower;
            }
        }

        leftLiftServo.setPower(power);
        rightLiftServo.setPower(power);
    }

    public void zero() {
        timer.reset();
        runningTimed = true;
        lift = false;
        timedPower = ZERO_POWER;
        timedMs = ZERO_TIME;
    }
    public void activate() {
        timer.reset();
        runningTimed = true;
        lift = true;
        timedPower = LIFT_POWER;
        timedMs = LIFT_TIME;
    }
}
