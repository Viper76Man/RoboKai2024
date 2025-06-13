package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpV3 extends OpMode {
    public RobotV2 robotv2 = new RobotV2();
    @Override
    public void init() {
        robotv2.init(hardwareMap, telemetry, RobotV2.Mode.TELEOP, gamepad1);
    }

    @Override
    public void loop() {
        //DRIVE
        robotv2.drive();
        robotv2.systemStatesUpdate();
        if(robotv2.isGamepadReady() && gamepad1.options){
            robotv2.toggleSlowMode();
            robotv2.resetButtonTimer();
        }
        else if(robotv2.isGamepadReady() && gamepad1.square){
            robotv2.nextState(1);
            robotv2.resetButtonTimer();
        }
        else if(robotv2.isGamepadReady() && gamepad1.triangle){
            robotv2.nextState(2);
            robotv2.resetButtonTimer();
        }
        robotv2.logStates();
    }
}