package org.firstinspires.ftc.teamcode.JackBurr.Odometry;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PinpointV1 {
    public GoBildaPinpointDriver pinpoint;
    public void init(HardwareMap hardwareMap){
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }
    public void update(){
        pinpoint.update();
    }
    public void resetPosAndIMU(){
        pinpoint.resetPosAndIMU();
    }
}
