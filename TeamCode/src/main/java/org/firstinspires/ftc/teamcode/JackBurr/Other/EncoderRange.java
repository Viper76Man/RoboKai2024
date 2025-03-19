package org.firstinspires.ftc.teamcode.JackBurr.Other;

public class EncoderRange {
    public double num1, num2;
    public double target;

    public EncoderRange(double target, double tolerance){
        this.num1 = target - tolerance;
        this.num2 = target + tolerance;
        this.target = target;
    }

    public boolean isInRange(double number){
        if(number >= num1 && number <= num2){
            return true;
        }
        else {
            return number == num1 || number == num2;
        }
    }

    public boolean isLeftOfRange(double number){
        return number < num1;
    }

    public boolean isRightOfRange(double number){
        return number > num2;
    }

    public double getTarget(){
        return this.target;
    }
}
