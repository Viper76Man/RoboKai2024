package org.firstinspires.ftc.teamcode.JackBurr.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorV1 {
    public DistanceSensor sensor;
    public HardwareMap hardwareMap;
    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        sensor = this.hardwareMap.get(DistanceSensor.class, "distance");
    }

    public double getDistance(DistanceUnit unit){
        return sensor.getDistance(unit);
    }
}
