package org.firstinspires.ftc.teamcode.JackBurr.Drive;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.IntakeSlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.GrippersV1;

public class Robot {
    //HARDWARE CLASSES
    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public DeliveryAxonV1 deliveryAxon = new DeliveryAxonV1();
    public IntakeSlidesV1 intakeSlides = new IntakeSlidesV1();
    public GrippersV1 grippers = new GrippersV1();
    public DeliveryGrippersV1 deliveryGrippers = new DeliveryGrippersV1();

    //VARIABLES
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public enum Mode {
        AUTO,
        TELEOP
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //Init all hardware classes
        slides.init(hardwareMap);
        deliveryGrippers.init(hardwareMap);
        intakeSlides.init(hardwareMap);
        deliveryAxon.init(hardwareMap);
    }

    public void init(Mode mode){
        switch (mode){
            case AUTO:
                zero();
                break;
            case TELEOP:
                telemetry.addLine("Entering TeleOp.");
                break;
        }
    }

    public void zero(){
        slides.resetSlides();
        intakeSlides.resetSlides();
    }
}
