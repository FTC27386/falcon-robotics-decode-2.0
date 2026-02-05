package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    VisionSubsystem v;
    DrivetrainSubsystem d;
    ShooterSubsystem s;
    GateSubsystem g;
    IntakeSubsystem i;
    LiftSubsystem l;


    public VisionSubsystem getV() {
        return v;
    }
    public DrivetrainSubsystem getD() {
        return d;
    }
    public ShooterSubsystem getS() {
        return s;
    }
    public GateSubsystem getG() {
        return g;
    }
    public IntakeSubsystem getI() {
        return i;
    }
    public LiftSubsystem getL() {
        return l;
    }


    public Robot(final HardwareMap hmap) {
        v = new VisionSubsystem(hmap);
        d = new DrivetrainSubsystem(hmap);
        s = new ShooterSubsystem(hmap);
        g = new GateSubsystem(hmap);
        i = new IntakeSubsystem(hmap);
        l = new LiftSubsystem(hmap);

        for (LynxModule mod : hmap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void periodic() {
        v.periodic();
        d.periodic();
        s.periodic();
        g.periodic();
        i.periodic();
        l.periodic();
    }

    public void setShooterValues() {
        s.setFlywheelSpeed(d.getFlywheel());
        s.setHoodAngle(d.getHood());
        s.setTurretAngle(d.getTurret());
    }
}
