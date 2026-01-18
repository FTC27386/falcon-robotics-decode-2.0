package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

import java.util.List;

public class VisionSystem extends SubsystemBase {

    private Limelight3A limelight;
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double METERS_TO_INCHES = 39.37;


    public VisionSystem(final HardwareMap hMap) {
        limelight = hMap.get(Limelight3A.class, RobotConstants.limelight_name);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0);
        limelight.start(); // This tells Limelight to start looking!
    }

    public void periodic() {

    }

    public void startLimelight(Telemetry telemetry, Pose currentPose) {
        double robotYawDeg = Math.toDegrees(currentPose.getHeading());
        limelight.updateRobotOrientation(robotYawDeg);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double llX_in = botpose_mt2.getPosition().x * METERS_TO_INCHES;
                double llY_in = botpose_mt2.getPosition().y * METERS_TO_INCHES;

                // Pedro frame: bottom-left origin
                double pedroX = llX_in + FIELD_SIZE_IN / 2.0; // forward
                double pedroY = llY_in + FIELD_SIZE_IN / 2.0; // right

                telemetry.addData("Pedro X", pedroX);
                telemetry.addData("Pedro Y", pedroY);
            }
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}
