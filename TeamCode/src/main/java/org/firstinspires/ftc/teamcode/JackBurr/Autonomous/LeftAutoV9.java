package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import androidx.core.view.TintableBackgroundView;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathCallback;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.IntakeSlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DifferentialV2;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.GrippersV1;

@Autonomous
@Config
public class LeftAutoV9 extends LinearOpMode {
    public Follower follower;
    public PoseUpdater poseUpdater;
    public boolean trajFollowed = false;
    public PathChain scorePreloadChain;
    public static Pose startPose = new Pose(38, 62, Math.toRadians(-90));
    public static Pose bucket = new Pose(50, 52, Math.toRadians(-135));
    public static Pose pickup1 = new Pose(48, 42.5, Math.toRadians(-93));
    public Path scorePreload;
    public Path pickup_1;
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

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        slides.init(hardwareMap);
        deliveryAxon.init(hardwareMap);
        intakeSlides.init(hardwareMap);
        grippers.init(hardwareMap, telemetry);
        differentialV2.init(hardwareMap, telemetry);
        deliveryGrippers.init(hardwareMap, telemetry);
        follower = new Follower(hardwareMap);
        poseUpdater = new PoseUpdater(hardwareMap);
        follower.setStartingPose(startPose);
        poseUpdater.setStartingPose(startPose);
        scorePreload = new Path(new BezierLine(new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN), new Point(bucket.getX(), bucket.getY(), Point.CARTESIAN)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), bucket.getHeading());
        pickup_1 = new Path(new BezierLine(new Point(bucket.getX(), bucket.getY(), Point.CARTESIAN), new Point(pickup1.getX(), pickup1.getY(), Point.CARTESIAN)));
        pickup_1.setLinearHeadingInterpolation(bucket.getHeading(), pickup1.getHeading());
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
                if(!trajFollowed) {
                    follower.followPath(scorePreload, true);
                    stepTimer.reset();
                    trajFollowed = true;
                }
                if(stepTimer.seconds() > 1){
                    follower.breakFollowing();
                    follower.setPose(bucket);
                    stepTimer.reset();
                    step = 2;
                }
            }
            if(step == 2){
                while (stepTimer.seconds() < 4){
                    follower.update();
                    if(stepTimer.seconds() > 2){
                        deliveryAxon.setPosition(constants.DELIVERY_UP);
                    }
                    if(stepTimer.seconds() < 3.5) {
                        telemetry.addLine("Moving slides up....");
                        slides.runRightSlideToPositionPID(constants.RIGHT_SLIDE_HIGH_BASKET);
                        slides.runLeftSlideToPositionPID(constants.LEFT_SLIDE_HIGH_BASKET);
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
                if(stepTimer.seconds() > 1){
                    stepTimer.reset();
                    step = 5;
                }
            }
            if(step == 5){
                if(stepTimer.seconds() < 2) {
                    slides.runLeftSlideToPositionPID(0);
                    slides.runRightSlideToPositionPID(0);
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
                Path path = new Path(new BezierLine(bucket, pickup1));
                path.setLinearHeadingInterpolation(bucket.getHeading(), pickup1.getHeading());
                PathChain aMetaphorForHowIDontGiveAShit = follower.pathBuilder()
                        .addPath(path)
                        .addTemporalCallback(0, ()->{
                            grippers.setPosition(constants.GRIPPERS_OPEN);
                            intakeSlides.intakeOutAuto();
                            differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_PICKUP);
                            differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_PICKUP);
                        })
                        .build();
                if(!trajFollowed) {
                    follower.followPath(aMetaphorForHowIDontGiveAShit, true);
                    trajFollowed = true;
                }
                stepTimer.reset();
                if(follower.atParametricEnd()) {
                    step = 7;
                    trajFollowed = false;
                }
            }
            if(step == 7){
                //follower.followPath();
                while (stepTimer.seconds() < 5) {
                    grippers.setPosition(constants.GRIPPERS_GRAB);
                }
                while (stepTimer.seconds() > 5 && stepTimer.seconds() < 6) {
                    differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                    differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                }
                step = 8;
            }


        }

    }

}
