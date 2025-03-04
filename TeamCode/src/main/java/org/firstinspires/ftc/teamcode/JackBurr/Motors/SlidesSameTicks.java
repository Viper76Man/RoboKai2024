package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
@TeleOp
public class SlidesSameTicks extends OpMode {
    public DcMotor leftSlide;
    public DcMotor rightSlide;
    public RobotConstantsV1 constants = new RobotConstantsV1();
    @Override
    public void init() {
        leftSlide = hardwareMap.get(DcMotor.class, "deliverySlideL");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide = hardwareMap.get(DcMotor.class, "deliverySlideR");
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        leftSlide.setPower(1);
        rightSlide.setPower(1);
        leftSlide.setTargetPosition(constants.LEFT_SLIDE_HIGH_BASKET);
        rightSlide.setTargetPosition(constants.RIGHT_SLIDE_HIGH_BASKET);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double error = leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition();
        telemetry.addLine("error: " + error);
        telemetry.addLine("LEFT POWER: " + leftSlide.getPower());
        telemetry.addLine("RIGHT POWER: " + rightSlide.getPower());
    }
}
