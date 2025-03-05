package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristAxonV1 {
    public Servo axon;
    public HardwareMap hardwareMap;
    public double position_;

    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        axon = hardwareMap.get(Servo.class, "wristServo");
    }

    public void setPosition(double position){
        axon.setPosition(position);
        this.position_ = position;
    }

    public double getServoPosition(){
        return axon.getPosition();
    }

    public double getPosition(){
        return position_;
    }

    public void moveLeft(double distance){
        distance = Math.abs(distance);
        if((position_ + distance) >= 1){
            this.position_ = 1;
            setPosition(1);
        }
        else {
            axon.setPosition(position_ + distance);
            this.position_ = position_ + distance;
        }
    }

    public void moveRight(double distance){
        distance = Math.abs(distance);
        if((position_ - distance) <= 0){
            this.position_ = 0;
            setPosition(0);
        }
        else {
            axon.setPosition(position_ - distance);
            this.position_ = position_ - distance;
        }
    }
}
