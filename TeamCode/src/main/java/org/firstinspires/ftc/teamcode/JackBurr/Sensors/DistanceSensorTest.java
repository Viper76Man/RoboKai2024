package org.firstinspires.ftc.teamcode.JackBurr.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorTest extends OpMode {
    public DistanceSensorV1 sensor = new DistanceSensorV1();
    @Override
    public void init() {
        sensor.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addLine("Distance: " + sensor.getDistance(DistanceUnit.INCH));
    }
}
