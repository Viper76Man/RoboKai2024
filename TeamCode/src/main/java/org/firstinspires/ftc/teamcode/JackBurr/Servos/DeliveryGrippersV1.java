package org.firstinspires.ftc.teamcode.JackBurr.Servos;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DeliveryGrippersV1 {
    public Servo grippers;
    public HardwareMap hardwareMap;
    public AnalogInput encoder;
    public void init(HardwareMap hwMap){
        this.hardwareMap = hwMap;
        grippers = hardwareMap.get(Servo.class, "deliveryGrippers");
        encoder = hardwareMap.get(AnalogInput.class, "deliveryGrippersEncoder");

    }

    public void setPosition(double position){
        grippers.setPosition(position);
    }

    public double getEncoderPosition(){
        return encoder.getVoltage() / 3.3 * 360;
    }
    public double getPosition(){
        return grippers.getPosition();
    }
}
