package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import android.view.contentcapture.DataRemovalRequest;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.IntakeSlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PinpointV1;
import org.firstinspires.ftc.teamcode.JackBurr.Other.EncoderRange;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DifferentialV2;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.GrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.WristAxonV1;

public class RobotV2 {
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
    public int SLIDES_RANGE_TOLERANCE = 40;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gamepad1;
    public ElapsedTime stateTimer = new ElapsedTime();
    public EncoderRange slidesDownRange = new EncoderRange(0, SLIDES_RANGE_TOLERANCE);


    //Delivery slide positions
    public int LEFT_SLIDE_DOWN = 0;
    public int RIGHT_SLIDE_DOWN = 0;
    public int LEFT_SLIDE_HIGH_BASKET = constants.LEFT_SLIDE_HIGH_BASKET;
    public int RIGHT_SLIDE_HIGH_BASKET = constants.RIGHT_SLIDE_HIGH_BASKET;
    public int LEFT_SLIDE_HIGH_BAR = constants.LEFT_SLIDE_HIGH_BAR;
    public int RIGHT_SLIDE_HIGH_BAR = constants.RIGHT_SLIDE_HIGH_BAR;

    //Auto or TeleOp mode
    public enum Mode {
        AUTO,
        TELEOP
    }

   //System States
   public enum SystemStates {
       START,
       //SAMPLES ONLY
       LOW_HOVER,
       DOWN_ON_SAMPLE, //Timers move to next state
       DOWN_ON_SAMPLE_GRAB, //TODO: Find put if Liam wants it to go straight from grab to transfer
       //SPECIMENS ONLY
       UNDER_LOW_BAR_HOVER,
       UNDER_LOW_BAR_SLIDES_OUT,
       DOWN_ON_SAMPLE_GRAB_2,
       UNDER_LOW_BAR_SLIDES_IN_HOVER,
       //Rest of the path
       DIFF_UP, //Timers move to next state
       SLIDES_IN,
       DELIVERY_GRIPPERS_CLOSED, //Timers move to next state
       INTAKE_GRIPPERS_OPEN, //Timers move to next state
       INTAKE_GRIPPERS_OPEN_SLIDES_OUT,
       DELIVER_HIGH_BAR,
       DELIVER_HIGH_BASKET,
       DROP_HIGH_BASKET,
       DELIVER_LOW_BASKET
   }

   //Helps decide where to go back to when triangle is pressed
   public enum StatesPath {
       NONE,
       SAMPLE,
       SPECIMEN
   }

    //Slowmode for Liam,
    //stateFinished to find out if the current state is complete to allow Liam to press the button to move on,
    //Button Timer
    //System States
    public boolean slowmode = false;
    public boolean stateFinished = false;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public SystemStates systemState = SystemStates.START;


    //Init all hardware and telemetry
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

    //Init all hardware classes
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
        grippers.init(hardwareMap);
    }

    //Zero both sets of slides
    public void zero(){
        slides.resetSlides();
        intakeSlides.resetSlides();
    }

    //Drive function
    public void drive(){
        if(slowmode){
            drivetrain.driveSlowMode(gamepad1);
        }
        else {
            drivetrain.drive(gamepad1);
        }
    }

    //Update the wrist, use a custom class i made to update the encoder ranges
    public void update(){
        updateWrist();
        if(slidesDownRange.getTarget() != LEFT_SLIDE_DOWN){
            slidesDownRange = new EncoderRange(LEFT_SLIDE_DOWN, SLIDES_RANGE_TOLERANCE);
        }
    }

    //Update wrist
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

    //Toggle slowmode
    public void toggleSlowMode(){
        this.slowmode = !slowmode;
    }

    //Check if gamepad is ready
    public boolean isGamepadReady(){
        return buttonTimer.seconds() > constants.BUTTON_PRESS_DELAY;
    }

    //Reset button timer
    public void resetButtonTimer(){
        buttonTimer.reset();
    }

    public void systemStatesUpdate(){
        switch (systemState){
            case START:
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                wrist.setPosition(constants.WRIST_CENTER);
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                if(stateTimer.seconds() > 1.5) {
                    if(slidesDownRange.isInRange(slides.getLeftSlidePosition())){
                        slides.runLeftSlideToPosition(LEFT_SLIDE_DOWN, 0.2);
                        slides.runRightSlideToPosition(RIGHT_SLIDE_DOWN, 0.2);
                        stateFinished = true;
                    }
                    else {
                        slides.runLeftSlideToPosition(LEFT_SLIDE_DOWN, 1);
                        slides.runRightSlideToPosition(RIGHT_SLIDE_DOWN, 1);
                    }
                    intakeSlides.intakeAllTheWayIn();
                }
                break;
            case LOW_HOVER:
                diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_LOW_HOVER);
                diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_LOW_HOVER);
                intakeSlides.intakeOut();
                grippers.setPosition(constants.GRIPPERS_OPEN);
                if(getStateTimerSeconds() > 1.5){
                    stateFinished = true;
                    break;
                }
                break;
            case DOWN_ON_SAMPLE:
                diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_PICKUP);
                diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_PICKUP);
                //TODO: Change this if we should wait to grab
                if (getStateTimerSeconds() > 0.65) {
                    setSystemState(SystemStates.DOWN_ON_SAMPLE_GRAB);
                    break;
                }
                break;
            case DOWN_ON_SAMPLE_GRAB:
                grippers.setPosition(constants.GRIPPERS_GRAB);
                if(getStateTimerSeconds() > 0.3){
                    setSystemState(SystemStates.DIFF_UP);
                    break;
                }
                break;
            case DIFF_UP:
                wrist.setPosition(constants.WRIST_CENTER);
                diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                if(getStateTimerSeconds() > 1){
                    setSystemState(SystemStates.SLIDES_IN);
                }
                break;
            case SLIDES_IN:
                intakeSlides.intakeAllTheWayIn();
                if(getStateTimerSeconds() > 0.75){
                    setSystemState(SystemStates.DELIVERY_GRIPPERS_CLOSED);
                }
                break;
            case DELIVERY_GRIPPERS_CLOSED:
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                if(getStateTimerSeconds() > 0.5){
                    setSystemState(SystemStates.INTAKE_GRIPPERS_OPEN);
                }
                break;
            case INTAKE_GRIPPERS_OPEN:
                grippers.setPosition(constants.GRIPPERS_OPEN);
                if(getStateTimerSeconds() > 0.5){
                    setSystemState(SystemStates.INTAKE_GRIPPERS_OPEN_SLIDES_OUT);
                }
                break;
            case INTAKE_GRIPPERS_OPEN_SLIDES_OUT:
                intakeSlides.intakeIn();
                if(getStateTimerSeconds() > 0.2){
                    stateFinished = true;
                }
                break;
            case DELIVER_HIGH_BASKET:
                slides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BASKET, 1);
                slides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BASKET, 1);
                if(Math.abs(slides.getLeftSlidePosition()) > (Math.abs(constants.LEFT_SLIDE_HIGH_BASKET) / 2)){
                    deliveryAxon.setPosition(constants.DELIVERY_UP);
                }
                if(stateTimer.seconds() > 1.75){
                    stateFinished = true;
                }
                break;
            case DROP_HIGH_BASKET:
                deliveryAxon.setPosition(constants.DELIVERY_DROP);
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                if(stateTimer.seconds() > 0.2){
                    stateFinished = true;
                }
                break;
        }
        update();
    }

    //Next system state
    public void nextState(int button){
        switch (button){
            case 1: //Square (Also known as X) (Sample path)
                if(stateFinished){
                    switch (systemState){
                        case START:
                            setSystemState(SystemStates.LOW_HOVER);
                            break;
                        case LOW_HOVER:
                            setSystemState(SystemStates.DOWN_ON_SAMPLE);
                            break;
                        case DOWN_ON_SAMPLE_GRAB:
                            setSystemState(SystemStates.DIFF_UP);
                            break;
                        case INTAKE_GRIPPERS_OPEN_SLIDES_OUT:
                            setSystemState(SystemStates.DELIVER_HIGH_BASKET);
                            break;
                        case DELIVER_HIGH_BASKET:
                            setSystemState(SystemStates.DROP_HIGH_BASKET);
                            break;
                        case DROP_HIGH_BASKET:
                            setSystemState(SystemStates.START);
                            break;
                    }
                    stateTimer.reset();
                }
                stateFinished = false;
                break;
            case 2: //Triangle (Reset button)
                //TODO: Add the reset button
                switch (systemState){
                    case LOW_HOVER:
                    case DROP_HIGH_BASKET:
                        setSystemState(SystemStates.START);
                        break;
                    case DOWN_ON_SAMPLE:
                    case DOWN_ON_SAMPLE_GRAB_2:
                    case DIFF_UP:
                    case SLIDES_IN:
                    case INTAKE_GRIPPERS_OPEN:
                    case INTAKE_GRIPPERS_OPEN_SLIDES_OUT:
                    case DELIVERY_GRIPPERS_CLOSED:
                    case DELIVER_HIGH_BASKET:
                        setSystemState(SystemStates.LOW_HOVER);
                        break;
                }
                break;

        }
    }

    //Set system state
    public void setSystemState(SystemStates systemState){
        //TODO: add state reset state finished (delivery) if needed
        this.systemState = systemState;
        stateTimer.reset();
    }

    //get state timer time
    public double getStateTimerSeconds(){
        return stateTimer.seconds();
    }

    //Log system states
    public void logStates(){
        telemetry.addLine("System State: " + systemState.name());
        if(stateFinished){
            telemetry.addLine("STATE FINISHED");
        }
    }


}
