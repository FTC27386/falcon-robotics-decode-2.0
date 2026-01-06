package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

public class Robot {
    shooterSystem s;
    intakeSystem i;
    drivetrainSystem d;
    liftSystem l;


    public shooterSystem getS() {
        return s;
    }

    public intakeSystem getI() {
        return i;
    }

    public drivetrainSystem getD() {
        return d;
    }

    public liftSystem getL() {
        return l;
    }

    public Robot(final HardwareMap hmap) {
        s = new shooterSystem(hmap);
        i = new intakeSystem(hmap);
        d = new drivetrainSystem(hmap);
        l = new liftSystem(hmap);
        for (LynxModule mod : hmap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void periodic() {
        i.periodic();
        s.periodic();
        d.periodic();
        l.periodic();
    }

    public void setShooterValues() {
        s.setSpeed(d.yoCalcSpeed());
        s.setHoodPosition(d.yoCalcHood());
        s.setTurretPosition(d.yoCalcAim());
    }

    public void setAutoValuesBlue() {
        s.setSpeed(-1620);
        s.setHoodPosition(0.24);
        s.setTurretPosition(48);
    }
    public void setAutoValuesRed() {
        s.setSpeed(-1620);
        s.setHoodPosition(0.24);
        //75
        s.setTurretPosition(-48.75);
    }
    public void setAutoValuesFarZoneBlue() {
        s.setSpeed(-2200);
        s.setHoodPosition(.95);
        s.setTurretPosition(67.3);
    }
    public void setAutoValuesFarZoneRed() {
        s.setSpeed(-2200);
        s.setHoodPosition(.95);
        s.setTurretPosition(-67.3);
    }

    public void sortShot() {

    }
}
