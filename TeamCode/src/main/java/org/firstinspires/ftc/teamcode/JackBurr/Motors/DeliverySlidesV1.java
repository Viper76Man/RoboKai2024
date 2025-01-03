package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeliverySlidesV1 {
    public HardwareMap hardwareMap;
    public DcMotor leftSlide;
    public DcMotor rightSlide;
    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        leftSlide = hardwareMap.get(DcMotor.class, "deliverySlideL");
        rightSlide = hardwareMap.get(DcMotor.class, "deliverySlideR");
    }

    public void runLeftSlideToPosition(int position, double power){
        leftSlide.setPower(power);
        leftSlide.setTargetPosition(position);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runRightSlideToPosition(int position, double power){
        rightSlide.setPower(power);
        rightSlide.setTargetPosition(position);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}