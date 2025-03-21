package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.IntakeSlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.FConstantsLeft;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DifferentialV2;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.GrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.WristAxonV1;
@Disabled
@Autonomous
@Config
public class LeftAutoV9 extends LinearOpMode {
    public Follower follower;
    public PoseUpdater poseUpdater;
    public boolean trajFollowed = false;
    public PathChain scorePreloadChain;
    public static Pose startPose = new Pose(38, 62, Math.toRadians(-90));
    public static Pose bucket = new Pose(50, 52, Math.toRadians(-135));
    public static Pose pickup1 = new Pose(48.5, 42.5, Math.toRadians(-90));
    public static Pose bucket2 = new Pose(51.5, 51,  Math.toRadians(-135));
    public Path scorePreload;
    public Path pickup_1;
    public Path delivery_02;
    public PathChain firstPickup;
    public Pose currentPose;

    public int step = 1;

    public ElapsedTime stepTimer = new ElapsedTime();

    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public IntakeSlidesV1 intakeSlides = new IntakeSlidesV1();
    public GrippersV1 grippers = new GrippersV1();
    public RobotConstantsV1 constants = new RobotConstantsV1();
    public DeliveryAxonV1 deliveryAxon = new DeliveryAxonV1();
    public DeliveryGrippersV1 deliveryGrippers = new DeliveryGrippersV1();
    public DifferentialV2 differentialV2 = new DifferentialV2();
    public WristAxonV1 wrist = new WristAxonV1();

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstantsLeft.class, LConstants.class);
        slides.init(hardwareMap);
        deliveryAxon.init(hardwareMap);
        intakeSlides.init(hardwareMap);
        wrist.init(hardwareMap);
        grippers.init(hardwareMap);
        differentialV2.init(hardwareMap);
        deliveryGrippers.init(hardwareMap);
        follower = new Follower(hardwareMap);
        poseUpdater = new PoseUpdater(hardwareMap);
        follower.setStartingPose(startPose);
        poseUpdater.setStartingPose(startPose);
        scorePreload = new Path(new BezierLine(new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN), new Point(bucket.getX(), bucket.getY(), Point.CARTESIAN)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), bucket.getHeading());
        pickup_1 = new Path(new BezierLine(bucket, pickup1));
        delivery_02 = new Path(new BezierLine(pickup1, bucket2));
        delivery_02.setLinearHeadingInterpolation(pickup1.getHeading(), bucket.getHeading());
        // THE HEADING IS AROUND 2000, change to fix?
        pickup_1.setLinearHeadingInterpolation(bucket.getHeading(), pickup1.getHeading());
        firstPickup = follower.pathBuilder()
                .addPath(pickup_1)
                .addTemporalCallback(0, ()->{
                    grippers.setPosition(constants.GRIPPERS_OPEN);
                    intakeSlides.intakeOutAuto();
                    differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_PICKUP);
                    differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_PICKUP);
                })
                .build();
        waitForStart();
        stepTimer.reset();
        while(opModeIsActive()) {
            currentPose = poseUpdater.getPose();
            if (isStopRequested()) {
                return;
            }
            follower.update();
            telemetry.update();
            poseUpdater.update();
            follower.drawOnDashBoard();
            telemetry.addLine("Step: " + step);
            telemetry.addLine("\t X: " + follower.getPose().getX());
            telemetry.addLine("\t Y: " + follower.getPose().getY());
            telemetry.addLine("\t Heading: " + Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addLine("\t Heading (poseUpdater): " + Math.toDegrees(poseUpdater.getPose().getHeading()));
            if (step == 1) {
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                if(!trajFollowed) {
                    follower.followPath(scorePreload, true);
                    stepTimer.reset();
                    trajFollowed = true;
                }
                if(stepTimer.seconds() > 1.3){
                    follower.breakFollowing();
                    //follower.setPose(bucket);
                    stepTimer.reset();
                    step = 2;
                }
            }
            if(step == 2){
                while (stepTimer.seconds() < 4){
                    if (isStopRequested()) {
                        return;
                    }
                    follower.update();
                    if(stepTimer.seconds() > 2){
                        deliveryAxon.setPosition(constants.DELIVERY_UP);
                    }
                    if(stepTimer.seconds() < 3.5) {
                        telemetry.addLine("Moving slides up....");
                        slides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BASKET, 1);
                        slides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BASKET, 1);
                    }
                    if(stepTimer.seconds() < 3.75 && stepTimer.seconds() > 3.2) {
                        deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                    }
                }
                step = 4;
                trajFollowed = false;
                stepTimer.reset();
            }
            if(step == 4){
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                grippers.setPosition(constants.GRIPPERS_OPEN);
                wrist.setPosition(constants.WRIST_CENTER);
                if(stepTimer.seconds() > 1){
                    stepTimer.reset();
                    step = 5;
                }
            }
            if(step == 5){
                if(stepTimer.seconds() < 2) {
                    slides.runLeftSlideToPosition(0, 1);
                    slides.runRightSlideToPosition(0,1 );
                    telemetry.addLine("Moving slides down....");
                }
                else {
                    step = 6;
                    trajFollowed = false;
                    stepTimer.reset();
                }

            }
            if(step == 6){
                //while (stepTimer.seconds() < 3) {
                    //follower.turnTo(pickup1.getHeading());
                //}
                if(!trajFollowed) {
                    follower.followPath(firstPickup, true);
                    trajFollowed = true;
                }
                stepTimer.reset();
                if(!follower.isBusy()) {
                    follower.breakFollowing();
                    //follower.setPose(pickup1);
                    stepTimer.reset();
                    step = 7;
                    trajFollowed = false;
                }
            }
            if(step == 7){
                //follower.followPath();
                while (stepTimer.seconds() < 4 && stepTimer.seconds() > 2) {
                    if (isStopRequested()) {
                    return;
                    }
                    grippers.setPosition(constants.GRIPPERS_GRAB);
                    wrist.setPosition(constants.WRIST_CENTER);
                }
                while (stepTimer.seconds() > 4 && stepTimer.seconds() < 6) {
                    if (isStopRequested()) {
                        return;
                    }
                    differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                    differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                    wrist.setPosition(constants.WRIST_CENTER);
                }
                while (stepTimer.seconds() > 6 && stepTimer.seconds() < 8) {
                    if (isStopRequested()) {
                        return;
                    }
                    intakeSlides.intakeAllTheWayIn();
                    wrist.setPosition(constants.WRIST_CENTER);
                }
                while (stepTimer.seconds() > 8 && stepTimer.seconds() < 9) {
                    if (isStopRequested()) {
                        return;
                    }
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                    wrist.setPosition(constants.WRIST_CENTER);
                }
                while (stepTimer.seconds() > 9 && stepTimer.seconds() < 10) {
                    if (isStopRequested()) {
                        return;
                    }
                    grippers.setPosition(constants.GRIPPERS_OPEN);
                }
                while (stepTimer.seconds() > 10){
                    if (isStopRequested()) {
                    return;
                    }
                    step = 8;
                    stepTimer.reset();
                    trajFollowed = false;
                    break;
                }
            }
            if(step == 8){
                if(!trajFollowed) {
                    follower.followPath(delivery_02, true);
                    trajFollowed = true;
                    stepTimer.reset();
                }
                if(!follower.isBusy()) {
                    if(stepTimer.seconds() > 1) {
                        follower.breakFollowing();
                        stepTimer.reset();
                        step = 9;
                        trajFollowed = false;
                    }
                }
            }

            if (step == 9){
                while (stepTimer.seconds() < 3) {
                    if (isStopRequested()) {
                        return;
                    }
                    slides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BASKET, 1);
                    slides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BASKET, 1);
                    if(stepTimer.seconds() > 2){
                        deliveryAxon.setPosition(constants.DELIVERY_UP);
                    }
                }
                while (stepTimer.seconds() < 5){
                    if (isStopRequested()) {
                        return;
                    }
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                }
                step = 10;
            }
            if(step == 10){
                while (stepTimer.seconds() < 1.5) {
                    if (isStopRequested()) {
                    return;
                    }
                    deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                }
                while (stepTimer.seconds() < 3) {
                    if (isStopRequested()) {
                    return;
                    }
                    slides.runLeftSlideToPosition(0, 1);
                    slides.runRightSlideToPosition(0, 1);
                }
                step = 11;
            }


        }

    }

}
