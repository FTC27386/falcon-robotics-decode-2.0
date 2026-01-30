package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    ShooterSystem s;
    IntakeSystem i;
    DrivetrainSystem d;
    LiftSystem l;
    VisionSystem v;


    public ShooterSystem getS() {
        return s;
    }

    public IntakeSystem getI() {
        return i;
    }

    public DrivetrainSystem getD() {
        return d;
    }
    public LiftSystem getL() {
        return l;
    }
    public VisionSystem getV() { return v; }

    public Robot(final HardwareMap hmap) {
        s = new ShooterSystem(hmap);
        i = new IntakeSystem(hmap);
        d = new DrivetrainSystem(hmap);
        l = new LiftSystem(hmap);
        v = new VisionSystem(hmap);
        for (LynxModule mod : hmap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void periodic() {
        i.periodic();
        s.periodic();
        d.periodic();
        l.periodic();
        v.periodic();
    }

    public void setShooterValues() {
        s.setSpeed(d.getSpeed());
        s.setHoodAngle(d.getHood());
        s.setTurretSetPoint(d.getAim());
    }

    public void setManualShooterValues(double speed, double hood) {
        s.setSpeed(speed);
        s.setHoodAngle(hood);
        s.setTurretSetPoint(d.getAim());
    }

    public void setAutoValuesBlue() {
        s.setSpeed(-1620);
        s.setHoodAngle(0.24);
        s.setTurretSetPoint(48);
    }
    public void setAutoValuesRed() {
        s.setSpeed(-1620);
        s.setHoodAngle(0.24);
        //75
        s.setTurretSetPoint(-48.75);
    }
    public void setAutoValuesFarZoneBlue() {
        s.setSpeed(-2200);
        s.setHoodAngle(.95);
        s.setTurretSetPoint(67.3);
    }
    public void setAutoValuesFarZoneRed() {
        s.setSpeed(-2200);
        s.setHoodAngle(.95);
        s.setTurretSetPoint(-67.3);
    }

    public void sortShot() {

    }
}
