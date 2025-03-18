package org.firstinspires.ftc.teamcode.JackBurr.OldFiles;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;

@Disabled
@Autonomous
public class LeftAutoV2 extends LinearOpMode {
    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public DeliveryAxonV1 deliveryAxonV1 = new DeliveryAxonV1();
    public RobotConstantsV1 robotConstantsV1 = new RobotConstantsV1();
    public DeliveryGrippersV1 deliveryGrippers = new DeliveryGrippersV1();
    public ElapsedTime deliveryTimer = new ElapsedTime();
    public int step = 0;
    public boolean servoSet = false;
    public boolean traj4Followed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        //Pick SampleMecanumDrive for dashboard and RRMecanumDrive for no dashboard
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        deliveryAxonV1.init(hardwareMap);
        deliveryGrippers.init(hardwareMap);
        slides.init(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //Pose2d startPose = new Pose2d(60, 0, Math.toRadians(180));
        //drive.setPoseEstimate(startPose);

        //Trajectory traj1 = drive.trajectoryBuilder(startPose)
                //.addDisplacementMarker(()->{
                  //  deliveryGrippers.setPosition(robotConstantsV1.DELIVERY_GRIPPERS_CLOSE);
                   // deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_GRAB);
                //})
                //.splineTo(new Vector2d(40, -15), Math.toRadians(135))
                //.build();

        //Trajectory traj4 = drive.trajectoryBuilder(traj1.end())
                //.back(10)
                //.build();

        //Trajectory traj2 = drive.trajectoryBuilder(traj4.end())
               // .splineTo(new Vector2d(0, -15), Math.toRadians(-60))
                //.build();

        //Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
               // .back(50)
               // .build();




        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (step == 0) {
                //drive.followTrajectory(traj1);
                deliveryTimer.reset();
                step =  1;
            }
            if (step == 1){
                while (deliveryTimer.seconds() < 3){
                    deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_GRAB);
                    slides.runLeftSlideToPosition(robotConstantsV1.LEFT_SLIDE_HIGH_BASKET, 0.9);
                    slides.runRightSlideToPosition(robotConstantsV1.RIGHT_SLIDE_HIGH_BASKET, 0.9);
                    if(!servoSet) {
                        deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_UP);
                        servoSet = true;
                    }
                    deliveryGrippers.setPosition(robotConstantsV1.DELIVERY_GRIPPERS_CLOSE);
                }
                while (deliveryTimer.seconds() < 5){
                    servoSet = false;
                    deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_UP);
                    //drive.followTrajectory(traj4);
                }
                while (deliveryTimer.seconds() < 10){
                    deliveryGrippers.setPosition(robotConstantsV1.DELIVERY_GRIPPERS_OPEN);
                }
                while (deliveryTimer.seconds() < 14){
                    if(!servoSet) {
                        deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_GRAB);
                        servoSet = true;
                    }
                    if(slides.getLeftSlidePosition() !=0 || slides.getRightSlidePosition() !=0) {
                        slides.runLeftSlideToPosition(0, 0.9);
                        slides.runRightSlideToPosition(0, 0.9);
                    }
                    else if(slides.getRightSlidePosition() !=0 && slides.getLeftSlidePosition() !=0) {
                        slides.runLeftSlideToPosition(0, 0.9);
                        slides.runRightSlideToPosition(0, 0.9);
                    }
                }
                while (deliveryTimer.seconds() > 13 && servoSet) {
                    telemetry.addLine("HELLO");
                    servoSet = false;
                    step = 2;
                }
            }
            if(step == 2) {
                //drive.followTrajectory(traj2);
                step = 3;
            }
            if(step == 3) {
                //drive.followTrajectory(traj3);
                step = 4;
            }
            if(step == 4) {
                deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_LEVEL_ONE_ASCENT);
                step = 5;
            }
        }
    }
}
