package org.firstinspires.ftc.teamcode.JackBurr.Camera.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "Sensor")
public class LimelightFPS extends OpMode {
    public LimelightV1 limelight;
    @Override
    public void init() {
        limelight = new LimelightV1();
        limelight.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        telemetry.addLine(limelight.getStatus().getName());
        telemetry.addLine("\t FPS: " + limelight.getFps());
    }
}
