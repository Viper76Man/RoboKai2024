package org.firstinspires.ftc.teamcode.JackBurr.Servos;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GrippersV1 {
    public Servo grippers;
    public HardwareMap hardwareMap;
    public void init(HardwareMap hwMap){
        this.hardwareMap = hwMap;
        grippers = hardwareMap.get(Servo.class, "grippers");
    }

    public void setPosition(double position){
        grippers.setPosition(position);
    }

    public double getPosition(){
        return grippers.getPosition();
    }
}
