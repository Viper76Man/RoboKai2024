package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Motors.ArmMotorV1;

@TeleOp
public class RobotV1 extends OpMode {
    public RobotV1Config config = new RobotV1Config();
    public ArmMotorV1 arm = new ArmMotorV1(hardwareMap, "arm");

    public DcMotor frontLeft; //PORT 3
    public DcMotor frontRight; //PORT 1
    public DcMotor backLeft; //PORT 2
    public DcMotor backRight; // PORT 0

    public double ARM_POWER = 0.4;
    public int MOVEMENT_DISTANCE = 25;
    public ElapsedTime armTimer = new ElapsedTime();


    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, config.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, config.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, config.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, config.BACK_RIGHT);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        run_motors();
        arm.setPower(gamepad1.left_stick_y);
        if (gamepad1.left_stick_y == 0){
            arm.hold(ARM_POWER);
        }
        if (armTimer.seconds() > 0.3){
            if (gamepad1.dpad_up){
                arm.moveTo(arm.getPos() + MOVEMENT_DISTANCE, ARM_POWER, true);
                armTimer.reset();
            }
            else if (gamepad1.dpad_down){
                arm.moveTo(arm.getPos() - MOVEMENT_DISTANCE, ARM_POWER, true);
                armTimer.reset();
            }
        }
    }

    public void run_motors(){
        if (config.ARE_MOTORS_UPSIDE_DOWN) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x; // Counteract imperfect strafing, if the back motors are facing downwards this should be negative
            double rx = -gamepad1.right_stick_x; //This is reversed for our turning
            drive(y, x, rx);
        }
        else {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            drive(y, x, rx);
        }

    }
    public void drive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y - x + rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}