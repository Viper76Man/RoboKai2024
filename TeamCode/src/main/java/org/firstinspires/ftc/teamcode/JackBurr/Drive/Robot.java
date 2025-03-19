package org.firstinspires.ftc.teamcode.JackBurr.Drive;


import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.StartableService;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.IntakeSlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PinpointV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DifferentialV2;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.GrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.WristAxonV1;

public class Robot {
    //HARDWARE CLASSES
    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public DeliveryAxonV1 deliveryAxon = new DeliveryAxonV1();
    public IntakeSlidesV1 intakeSlides = new IntakeSlidesV1();
    public DifferentialV2 diffV2 = new DifferentialV2();
    public WristAxonV1 wrist = new WristAxonV1();
    public GrippersV1 grippers = new GrippersV1();
    public DeliveryGrippersV1 deliveryGrippers = new DeliveryGrippersV1();
    public PinpointV1 pinpoint = new PinpointV1();
    public Drivetrain drivetrain = new Drivetrain();
    public RobotConstantsV1 constants = new RobotConstantsV1();

    //VARIABLES
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gamepad1;
    public ElapsedTime stateTimer = new ElapsedTime();

    public enum DeliveryStates {
        TRANSFER_GRIPPERS_OPEN,
        TRANSFER_GRIPPERS_CLOSED,
        SLIDES_UP_HIGH_BAR,
        SLIDES_UP_HIGH_BASKET,
        DROP,
        RESET
    }

    public enum IntakeStates {
        START,
        REGULAR,
        UNDER_LOW_BAR,
        RESET
    }

    public enum RegularIntakeStates {
        START,
        LOW_HOVER,
        DOWN_ON_SAMPLE,
        DOWN_ON_SAMPLE_GRIPPERS_CLOSED,
        DIFF_UP,
        GRIPPERS_CLOSED_TRANSFER,
        GRIPPERS_OPEN,
        RESET
    }

    public enum UnderLowBarIntakeStates {
        START,
        UNDER_LOW_BAR,
        UNDER_LOW_BAR_SLIDES_OUT,
        UNDER_LOW_BAR_GRIPPERS_HOVER_LOW,
        UNDER_LOW_BAR_GRIPPERS_DOWN,
        UNDER_LOW_BAR_GRAB,
        UNDER_LOW_BAR_UP,
        UNDER_LOW_BAR_SLIDES_IN,
        RESET
    }

    public boolean slowmode = false;
    public boolean stateFinishedIntake = false;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public DeliveryStates deliveryState = DeliveryStates.TRANSFER_GRIPPERS_OPEN;
    public IntakeStates intakeState = IntakeStates.START;
    public RegularIntakeStates regularIntakeState = RegularIntakeStates.START;
    public UnderLowBarIntakeStates underLowBarIntakeStates = UnderLowBarIntakeStates.START;

    public enum Mode {
        AUTO,
        TELEOP
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, Mode mode, Gamepad gamepad1){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        initHardwareClasses();
        switch (mode){
            case AUTO:
                zero();
                pinpoint.resetPosAndIMU();
                break;
            case TELEOP:
                break;
        }
    }

    public void initHardwareClasses(){
        //Init all hardware classes
        slides.init(hardwareMap);
        deliveryGrippers.init(hardwareMap);
        intakeSlides.init(hardwareMap);
        deliveryAxon.init(hardwareMap);
        pinpoint.init(hardwareMap);
        drivetrain.init(hardwareMap);
        wrist.init(hardwareMap);
        diffV2.init(hardwareMap);
    }

    public void zero(){
        slides.resetSlides();
        intakeSlides.resetSlides();
    }

    public void drive(){
        if(slowmode){
            drivetrain.driveSlowMode(gamepad1);
        }
        else {
            drivetrain.drive(gamepad1);
        }
    }

    public void updateWrist(){
        if (gamepad1.left_bumper && isGamepadReady()){
            wrist.moveLeft(0.2);
            resetButtonTimer();

        }
        else if (gamepad1.right_bumper && isGamepadReady()){
            wrist.moveRight(0.2);
            resetButtonTimer();
        }
    }

    public void toggleSlowMode(){
        this.slowmode = !slowmode;
    }

    public boolean isGamepadReady(){
        return buttonTimer.seconds() > constants.BUTTON_PRESS_DELAY;
    }

    public void resetButtonTimer(){
        buttonTimer.reset();
    }

    public void systemStatesUpdate(){
        switch (intakeState){
            case REGULAR:
                switch (regularIntakeState){
                    case START:
                        intakeSlides.intakeIn();
                        diffV2.diffTransfer();
                        wrist.setPosition(constants.WRIST_CENTER);
                        stateFinishedIntake = true;
                        break;
                    case LOW_HOVER:
                        grippers.setPosition(constants.GRIPPERS_OPEN);
                        intakeSlides.intakeOut();
                        diffV2.diffHover();
                        updateWrist();
                        stateFinishedIntake = true;
                        break;
                    case DOWN_ON_SAMPLE:
                        diffV2.diffPickup();
                        updateWrist();
                        stateFinishedIntake = true;
                        break;
                    case DOWN_ON_SAMPLE_GRIPPERS_CLOSED:
                        grippers.setPosition(constants.GRIPPERS_GRAB);
                        if(stateTimer.seconds() > 0.4){
                            setRegularIntakeState(RegularIntakeStates.DIFF_UP);
                        }
                        break;
                    case DIFF_UP:
                        wrist.setPosition(constants.WRIST_CENTER);
                        diffV2.diffTransfer();
                        if(getStateTimerSeconds() > 0.5){
                            setRegularIntakeState(RegularIntakeStates.GRIPPERS_CLOSED_TRANSFER);
                            stateFinishedIntake = true;
                            stateTimer.reset();
                        }
                        break;
                    case GRIPPERS_CLOSED_TRANSFER:
                        wrist.setPosition(constants.WRIST_CENTER);
                        diffV2.diffTransfer();
                        intakeSlides.intakeAllTheWayIn();
                        if(getStateTimerSeconds() > 1.2 && getStateTimerSeconds() > 3){
                            setDeliveryState(DeliveryStates.TRANSFER_GRIPPERS_CLOSED);
                            stateTimer.reset();
                        }
                        else if(getStateTimerSeconds() > 3){
                            setRegularIntakeState(RegularIntakeStates.GRIPPERS_OPEN);
                        }
                        break;
                    case GRIPPERS_OPEN:
                        grippers.setPosition(constants.GRIPPERS_OPEN);
                        intakeSlides.intakeIn();
                        break;
                }
                break;
            case UNDER_LOW_BAR:
                break;
        }
        switch (deliveryState){
            case TRANSFER_GRIPPERS_OPEN:
                slides.runLeftSlideToPosition(0, 0.2);
                slides.runRightSlideToPosition(0, 0.2);
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                break;
            case TRANSFER_GRIPPERS_CLOSED:
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                break;
            case SLIDES_UP_HIGH_BASKET:
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                slides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BASKET,1);
                slides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BASKET,1);
                if(stateTimer.seconds() > 1){
                    deliveryAxon.setPosition(constants.DELIVERY_UP);
                }
                break;
            case SLIDES_UP_HIGH_BAR:
                deliveryAxon.setPosition(constants.DELIVERY_HIGH_BAR);
                slides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BAR,1);
                slides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BAR,1);
                break;
            case DROP:
                deliveryAxon.setPosition(constants.GRIPPERS_OPEN);

        }
    }

    public void nextState(){
        //Reduce driver error
        if(stateFinishedIntake){
            switch(intakeState) {
                case REGULAR:
                    switch (regularIntakeState) {
                        case START:
                            setRegularIntakeState(RegularIntakeStates.LOW_HOVER);
                            break;
                        case LOW_HOVER:
                            setRegularIntakeState(RegularIntakeStates.DOWN_ON_SAMPLE);
                            break;
                        case DOWN_ON_SAMPLE:
                            setRegularIntakeState(RegularIntakeStates.DOWN_ON_SAMPLE_GRIPPERS_CLOSED);
                            break;
                    }
                    break;
                case UNDER_LOW_BAR:
                    break;
            }
        }
        stateTimer.reset();
    }

    public void setIntakeState(IntakeStates intakeState){
        this.intakeState = intakeState;
    }

    public void setDeliveryState(DeliveryStates deliveryState){
        this.deliveryState = deliveryState;
    }

    public void setRegularIntakeState(RegularIntakeStates intakeState){
        stateFinishedIntake = false;
        this.regularIntakeState = intakeState;
    }

    public void setUnderLowBarIntakeState(UnderLowBarIntakeStates intakeState){
        this.underLowBarIntakeStates = intakeState;
    }

    public double getStateTimerSeconds(){
        return stateTimer.seconds();
    }


}
