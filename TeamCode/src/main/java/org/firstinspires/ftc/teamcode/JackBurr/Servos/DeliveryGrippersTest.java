package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DeliveryGrippersTest extends OpMode {
    public DeliveryGrippersV1 grippers = new DeliveryGrippersV1();
    public ElapsedTime buttonTimer = new ElapsedTime();
    public double target = 0;
    @Override
    public void init() {
        grippers.init(hardwareMap);
        grippers.setPosition(0);
    }

    @Override
    public void loop() {
        if (buttonTimer.seconds() > 0.3 && gamepad1.dpad_right){
            target = target + 0.05;
            buttonTimer.reset();
        }
        else if (buttonTimer.seconds() > 0.3 && gamepad1.dpad_left){
            target = target - 0.05;
            buttonTimer.reset();
        }
        grippers.setPosition(target);
        telemetry.addLine(String.valueOf(target));
        telemetry.addLine(String.valueOf(grippers.getEncoderPosition()));
    }
}
