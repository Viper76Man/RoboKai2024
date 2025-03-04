package org.firstinspires.ftc.teamcode.JackBurr.Other;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class MagneticLimitSwitchTest extends OpMode {
    public DigitalChannel limitSwitch;
    @Override
    public void init() {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        telemetry.addLine(String.valueOf(limitSwitch.getState()));
    }
}
