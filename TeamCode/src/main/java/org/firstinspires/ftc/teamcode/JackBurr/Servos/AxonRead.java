package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp
public class AxonRead extends OpMode {
    public AnalogInput servo_encoder;
    public Servo servo_;
    @Override
    public void init() {
        servo_ = hardwareMap.get(Servo.class, "left_diff");
        servo_encoder = hardwareMap.get(AnalogInput.class, "left_servo_encoder");
        servo_.setPosition(1);
    }

    @Override
    public void loop() {
        telemetry.addData("Servo Position: ", servo_.getPosition());
        telemetry.addData("Encoder: ", servo_encoder.getVoltage() / 3.3 * 360);
    }
}
