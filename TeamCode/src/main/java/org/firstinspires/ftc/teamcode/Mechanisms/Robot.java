package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    ShooterSystem s;
    IntakeSystem i;
    DrivetrainSystem d;
    LiftSystem l;
    VisionSystem v;

    public VisionSystem getV() { return v; }
    public DrivetrainSystem getD() {
        return d;
    }
    public ShooterSystem getS() {
        return s;
    }
    public IntakeSystem getI() {
        return i;
    }
    public LiftSystem getL() {
        return l;
    }


    public Robot(final HardwareMap hmap) {
        v = new VisionSystem(hmap);
        d = new DrivetrainSystem(hmap);
        s = new ShooterSystem(hmap);
        i = new IntakeSystem(hmap);
        l = new LiftSystem(hmap);

        for (LynxModule mod : hmap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void periodic() {
        v.periodic();
        d.periodic();
        s.periodic();
        i.periodic();
        l.periodic();
    }

    public void setShooterValues() {
        s.setFlywheelSpeed(d.getFlywheel());
        s.setHoodAngle(d.getHood());
        s.setTurretPosition(d.getTurret());
    }
}
