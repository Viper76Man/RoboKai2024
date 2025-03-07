package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstantsV1 {
    //CONFIG=========================================================================================================
    //Port 0

    //GRIPPERS=======================================================================================================
    public double GRIPPERS_OPEN = 0.45;
    public double GRIPPERS_CLOSE = 0;
    public double GRIPPERS_GRAB = 0.05;
    public double DELIVERY_GRIPPERS_CLOSE = 0.9;
    public double DELIVERY_GRIPPERS_GRAB = 0.47;
    public double DELIVERY_GRIPPERS_OPEN = 0.25;
    //===============================================================================================================
    //SERVOS=========================================================================================================
    public double FRONT_LEFT_TRANSFER = 0.9;
    public double FRONT_RIGHT_TRANSFER = 0.9;
    public double FRONT_LEFT_PICKUP = 0.03;
    public double FRONT_RIGHT_PICKUP = 0.03;
    public double FRONT_LEFT_LOW_HOVER = 0.095;
    public double FRONT_RIGHT_LOW_HOVER = 0.095;
    public double FRONT_LEFT_HOVER = 0.16;
    public double FRONT_RIGHT_HOVER = 0.16;
    public double FRONT_LEFT_OVER_LOW_BAR = 0.25;
    public double FRONT_RIGHT_OVER_LOW_BAR = 0.25;
    public double DELIVERY_DOWN = 1;
    public double DELIVERY_GRAB = 0.02; // 0.105 (-0.085)
    public double DELIVERY_LEVEL_ONE_ASCENT = 0.415; //0.5
    public double DELIVERY_LEVEL_TWO_ASCENT = 0.47; //0.555
    public double DELIVERY_UP = 0.47; //0.555
    public double DELIVERY_DROP = 0.50; //0.585
    public double DELIVERY_HIGH_BAR = 0.475; //0.56
    public double DELIVERY_HOOK = 0.675; //0.76
    public double DELIVERY_WALL_PICKUP = 0.69; //0.755
    public double WRIST_CENTER = 0.21;
    //MOTORS=========================================================================================================
    public int INTAKE_MOTOR_IN = 3;
    public int INTAKE_MOTOR_OUT = 850;
    public int INTAKE_MOTOR_ALL_THE_WAY_IN = 0;
    public int LEFT_SLIDE_HIGH_BASKET = -3195;
    public int RIGHT_SLIDE_HIGH_BASKET = 3195;
    public int LEFT_SLIDE_LOW_BASKET = -1582;
    public int RIGHT_SLIDE_LOW_BASKET = 1582;
    public int LEFT_SLIDE_HIGH_BAR = -468;
    public int RIGHT_SLIDE_HIGH_BAR = 468;
    public int LEFT_SLIDE_HIGH_BAR_AUTO = -468;
    public int RIGHT_SLIDE_HIGH_BAR_AUTO = 468;
    public int LEFT_SLIDE_LEVEL_TWO_ASCENT = -3370;
    public int RIGHT_SLIDE_LEVEL_TWO_ASCENT = 3370;
    public int LEFT_SLIDE_LEVEL_TWO_ASCENT_HOOK = -629;
    public int RIGHT_SLIDE_LEVEL_TWO_ASCENT_HOOK = 629;
    //CAMERA========================================================================================================
    public static double sampleAngle = 0;
    //==============================================================================================================
}
