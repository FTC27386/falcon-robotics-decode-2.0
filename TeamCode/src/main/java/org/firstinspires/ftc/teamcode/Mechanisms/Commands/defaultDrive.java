package org.firstinspires.ftc.teamcode.Mechanisms.Commands;

import static org.firstinspires.ftc.teamcode.Utility.UtilMethods.squareMagnitude;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

import java.util.function.Supplier;

public class defaultDrive extends CommandBase {
    Supplier<Double> axial, lateral, yaw;
    Robot r;

    public defaultDrive(Robot r, Supplier<Double> axial, Supplier<Double> lateral, Supplier<Double> yaw) {
        this.axial = axial;
        this.lateral = lateral;
        this.yaw = yaw;
        this.r = r;
        addRequirements(r.getD());
    }

    @Override
    public void execute() {
        r.getD().teleOpDrive(squareMagnitude(-axial.get()), squareMagnitude(lateral.get()), squareMagnitude(yaw.get()));
    }

}
