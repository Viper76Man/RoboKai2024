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

    //Delivery slide positions and tolerance
    public int SLIDES_RANGE_TOLERANCE = 200;
    public int LEFT_SLIDE_DOWN = 0;
    public int RIGHT_SLIDE_DOWN = 0;
    public int LEFT_SLIDE_HIGH_BASKET = constants.LEFT_SLIDE_HIGH_BASKET;
    public int RIGHT_SLIDE_HIGH_BASKET = constants.RIGHT_SLIDE_HIGH_BASKET;
    public int LEFT_SLIDE_LOW_BASKET = constants.LEFT_SLIDE_LOW_BASKET;
    public int RIGHT_SLIDE_LOW_BASKET = constants.RIGHT_SLIDE_LOW_BASKET;
    public int LEFT_SLIDE_HIGH_BAR = constants.LEFT_SLIDE_HIGH_BAR;
    public int RIGHT_SLIDE_HIGH_BAR = constants.RIGHT_SLIDE_HIGH_BAR;

    //VARIABLES
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gamepad1;
    public ElapsedTime stateTimer = new ElapsedTime();
    public EncoderRange slidesDownRange = new EncoderRange(0, SLIDES_RANGE_TOLERANCE);
    public int lastButtonPressed = 0;


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
       //LOW BAR ONLY
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
       DELIVER_LOW_BASKET,
       DROP_LOW_BASKET
   }

   //Helps decide where to go back to when triangle is pressed
   public enum StatesPath {
       NONE,
       REGULAR,
       UNDER_LOW_BAR
   }

    //Slowmode for Liam,
    //stateFinished to find out if the current state is complete to allow Liam to press the button to move on,
    //Button Timer
    //System States
    public boolean slowmode = false;
    public boolean stateFinished = false;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public SystemStates systemState = SystemStates.START;
    public StatesPath statesPath = StatesPath.NONE;

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
        //jog
        jog();
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

    public void slidesDownUpdate(){
        if(slidesDownRange.isInRange(slides.getLeftSlidePosition())){
            slides.runLeftSlideToPosition(LEFT_SLIDE_DOWN, 0.2);
            slides.runRightSlideToPosition(RIGHT_SLIDE_DOWN, 0.2);
            stateFinished = true;
        }
        else {
            slides.runLeftSlideToPosition(LEFT_SLIDE_DOWN, 1);
            slides.runRightSlideToPosition(RIGHT_SLIDE_DOWN, 1);
        }
    }

    public void systemStatesUpdate(){
        switch (systemState){
            case START:
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                wrist.setPosition(constants.WRIST_CENTER);
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                if(stateTimer.seconds() > 0.5) {
                    slidesDownUpdate();
                    intakeSlides.intakeIn();
                }
                break;
            case LOW_HOVER:
                statesPath = StatesPath.REGULAR;
                if (lastButtonPressed == 2 && stateTimer.seconds() > 1){
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                }
                else if(lastButtonPressed != 2) {
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                }
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_LOW_HOVER);
                diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_LOW_HOVER);
                intakeSlides.intakeOut();
                slidesDownUpdate();
                grippers.setPosition(constants.GRIPPERS_OPEN);
                if(getStateTimerSeconds() > 1.5){
                    stateFinished = true;
                    break;
                }
                break;
            case DOWN_ON_SAMPLE:
                diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_PICKUP);
                diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_PICKUP);
                //TODO: Change this if we should wait to grab'
                if(stateTimer.seconds() > 0.35){
                    stateFinished = true;
                }
                if(stateTimer.seconds() > 0.35 && lastButtonPressed == 1){
                    setSystemState(SystemStates.DOWN_ON_SAMPLE_GRAB);
                }
                break;
            case DOWN_ON_SAMPLE_GRAB:
                grippers.setPosition(constants.GRIPPERS_GRAB);
                if(stateTimer.seconds() > 0.3) {
                    setSystemState(SystemStates.DIFF_UP);
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
                if(Math.abs(slides.getLeftSlidePosition()) > (Math.abs(constants.LEFT_SLIDE_HIGH_BASKET + LEFT_SLIDE_DOWN)  / 2)){
                    deliveryAxon.setPosition(constants.DELIVERY_UP);
                }
                if(stateTimer.seconds() > 1.75){
                    stateFinished = true;
                }
                break;
            case DELIVER_LOW_BASKET:
                slides.runRightSlideToPosition(RIGHT_SLIDE_LOW_BASKET, 1);
                slides.runLeftSlideToPosition(LEFT_SLIDE_LOW_BASKET, 1);
                if(Math.abs(slides.getLeftSlidePosition()) > (Math.abs(LEFT_SLIDE_LOW_BASKET + LEFT_SLIDE_DOWN) / 2)){
                    deliveryAxon.setPosition(constants.DELIVERY_UP);
                }
                if(stateTimer.seconds() > 1.75){
                    stateFinished = true;
                }
                break;
            case DROP_LOW_BASKET:
            case DROP_HIGH_BASKET:
                deliveryAxon.setPosition(constants.DELIVERY_DROP);
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                if(stateTimer.seconds() > 0.3){
                    setSystemState(SystemStates.START);
                }
                break;
            case UNDER_LOW_BAR_HOVER:
                statesPath = StatesPath.UNDER_LOW_BAR;
                grippers.setPosition(constants.GRIPPERS_OPEN);
                diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_HOVER);
                diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_HOVER);
                if(stateTimer.seconds() > 0.3){
                    stateFinished = true;
                }
                break;
            case UNDER_LOW_BAR_SLIDES_OUT:
                intakeSlides.intakeOut();
                if(stateTimer.seconds() > 0.3){
                    stateFinished = true;
                }
                break;
            case DOWN_ON_SAMPLE_GRAB_2:
                diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_PICKUP);
                diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_PICKUP);
                if(stateTimer.seconds() > 0.5){
                    grippers.setPosition(constants.GRIPPERS_GRAB);
                }
                if(stateTimer.seconds() > 0.8 && lastButtonPressed == 1){
                    setSystemState(SystemStates.UNDER_LOW_BAR_SLIDES_IN_HOVER);
                }
                else if(stateTimer.seconds() > 0.8 && lastButtonPressed == 3){
                    stateFinished = true;
                }
                break;
            case UNDER_LOW_BAR_SLIDES_IN_HOVER:
                diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_HOVER);
                diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_HOVER);
                if(stateTimer.seconds() > 0.5){
                    intakeSlides.intakeIn();
                }
                if(stateTimer.seconds() > 0.3){
                    stateFinished = true;
                }
                break;
        }
        update();
    }

    //Next system state
    public void nextState(int button){
        lastButtonPressed = button;
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
                        case UNDER_LOW_BAR_SLIDES_IN_HOVER:
                            setSystemState(SystemStates.DIFF_UP);
                            break;
                        case INTAKE_GRIPPERS_OPEN_SLIDES_OUT:
                            setSystemState(SystemStates.DELIVER_HIGH_BASKET);
                            break;
                        case DELIVER_HIGH_BASKET:
                            setSystemState(SystemStates.DROP_HIGH_BASKET);
                            break;
                        case UNDER_LOW_BAR_HOVER:
                            setSystemState(SystemStates.UNDER_LOW_BAR_SLIDES_OUT);
                            break;
                        case UNDER_LOW_BAR_SLIDES_OUT:
                            setSystemState(SystemStates.DOWN_ON_SAMPLE_GRAB_2);
                            break;
                        case DOWN_ON_SAMPLE_GRAB_2:
                            setSystemState(SystemStates.UNDER_LOW_BAR_SLIDES_IN_HOVER);
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
                    case UNDER_LOW_BAR_HOVER:
                    case DROP_HIGH_BASKET:
                        setSystemState(SystemStates.START);
                        break;
                    case UNDER_LOW_BAR_SLIDES_IN_HOVER:
                    case UNDER_LOW_BAR_SLIDES_OUT:
                    case DOWN_ON_SAMPLE_GRAB_2:
                        setSystemState(SystemStates.UNDER_LOW_BAR_HOVER);
                        break;
                    case START:
                        break;
                    default:
                        setSystemState(SystemStates.LOW_HOVER);
                        break;
                }
                break;
            case 3:
                switch (systemState){
                    case START:
                        setSystemState(SystemStates.UNDER_LOW_BAR_HOVER);
                        break;
                    case INTAKE_GRIPPERS_OPEN_SLIDES_OUT:
                        setSystemState(SystemStates.DELIVER_LOW_BASKET);
                        break;
                    case UNDER_LOW_BAR_SLIDES_OUT:
                        setSystemState(SystemStates.DOWN_ON_SAMPLE_GRAB_2);
                        break;
                    case LOW_HOVER:
                        setSystemState(SystemStates.DOWN_ON_SAMPLE);
                        break;

                }

        }
    }

    //Set system state
    public void setSystemState(SystemStates systemState){
        //TODO: add state reset state finished (delivery) if needed
        this.systemState = systemState;
        stateTimer.reset();
    }

    public void jog(){
        if(isGamepadReady() && gamepad1.dpad_down){
            switch (systemState){
                case DELIVER_HIGH_BASKET:
                    LEFT_SLIDE_HIGH_BASKET = LEFT_SLIDE_HIGH_BASKET + 20;
                    RIGHT_SLIDE_HIGH_BASKET = RIGHT_SLIDE_HIGH_BASKET - 20;
                    break;
                case DELIVER_LOW_BASKET:
                    LEFT_SLIDE_LOW_BASKET = LEFT_SLIDE_LOW_BASKET + 20;
                    RIGHT_SLIDE_LOW_BASKET = RIGHT_SLIDE_LOW_BASKET - 20;
                    break;
                case START:
                    LEFT_SLIDE_DOWN = LEFT_SLIDE_DOWN + 20;
                    RIGHT_SLIDE_DOWN = RIGHT_SLIDE_DOWN - 20;
                    break;
            }
            resetButtonTimer();
        }
        else if(isGamepadReady() && gamepad1.dpad_up){
            switch (systemState){
                case DELIVER_HIGH_BASKET:
                    LEFT_SLIDE_HIGH_BASKET = LEFT_SLIDE_HIGH_BASKET - 20;
                    RIGHT_SLIDE_HIGH_BASKET = RIGHT_SLIDE_HIGH_BASKET + 20;
                    break;
                case DELIVER_LOW_BASKET:
                    LEFT_SLIDE_LOW_BASKET = LEFT_SLIDE_LOW_BASKET - 20;
                    RIGHT_SLIDE_LOW_BASKET = RIGHT_SLIDE_LOW_BASKET + 20;
                    break;
                case START:
                    LEFT_SLIDE_DOWN = LEFT_SLIDE_DOWN - 20;
                    RIGHT_SLIDE_DOWN = RIGHT_SLIDE_DOWN + 20;
                    break;
            }
            resetButtonTimer();
        }
    }

    //get state timer time
    public double getStateTimerSeconds(){
        return stateTimer.seconds();
    }

    public void resetStateTimer(){
        stateTimer.reset();
    }

    //Log system states
    public void logStates(){
        telemetry.addLine("System State: " + systemState.name());
        telemetry.addLine();
        telemetry.addLine("Left Position: " + slides.getLeftSlidePosition());
        telemetry.addLine("Left Target: " + LEFT_SLIDE_DOWN);
        telemetry.addLine("Right Position: " + slides.getRightSlidePosition());
        telemetry.addLine("Right Target: " + RIGHT_SLIDE_DOWN);
        telemetry.addLine("Difference: " + (Math.abs(slides.getLeftSlidePosition()) - Math.abs(slides.getRightSlidePosition())));
        if(stateFinished){
            telemetry.addLine("STATE FINISHED");
        }
    }


}
