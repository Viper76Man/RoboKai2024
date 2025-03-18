package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpV3 extends OpMode {
    public Robot robot = new Robot();
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, Robot.Mode.TELEOP, gamepad1);
    }

    @Override
    public void loop() {
        //DRIVE
        robot.drive();
        robot.systemStatesUpdate();
        if(robot.isGamepadReady() && gamepad1.options){
            robot.toggleSlowMode();
            robot.resetButtonTimer();
        }
        else if(robot.isGamepadReady() && gamepad1.x){
            robot.nextState();
            robot.resetButtonTimer();
        }
        //STATES
    }
}