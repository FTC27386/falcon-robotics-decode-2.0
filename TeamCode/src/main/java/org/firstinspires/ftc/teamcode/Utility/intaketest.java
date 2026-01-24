package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class intaketest extends OpMode {

    public static int intakepower=0;
    public DcMotor intake;

    @Override
    public void init()
    {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop()
    {
        intake.setPower(intakepower);
    }


}
