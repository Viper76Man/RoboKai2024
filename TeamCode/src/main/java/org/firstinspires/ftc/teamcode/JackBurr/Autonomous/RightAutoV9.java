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
import org.firstinspires.ftc.teamcode.JackBurr.Servos.WristAxonV1;

@Autonomous
@Config
public class RightAutoV9 extends LinearOpMode {
    public Follower follower;
    public PoseUpdater poseUpdater;


    public static Pose startPose = new Pose(9, 63, Math.toRadians(180));
    public static Pose highBar = new Pose(39, 63, Math.toRadians(180));
    public PathChain scorePreloadChain;
    public Path scorePreload;

    public ElapsedTime stepTimer = new ElapsedTime();

    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public IntakeSlidesV1 intakeSlides = new IntakeSlidesV1();
    public GrippersV1 grippers = new GrippersV1();
    public RobotConstantsV1 constants = new RobotConstantsV1();
    public DeliveryAxonV1 deliveryAxon = new DeliveryAxonV1();
    public DeliveryGrippersV1 deliveryGrippers = new DeliveryGrippersV1();
    public DifferentialV2 differentialV2 = new DifferentialV2();
    public WristAxonV1 wrist = new WristAxonV1();

    public int step = 1;
    public boolean trajFollowed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        slides.init(hardwareMap);
        deliveryAxon.init(hardwareMap);
        intakeSlides.init(hardwareMap);
        wrist.init(hardwareMap);
        grippers.init(hardwareMap, telemetry);
        differentialV2.init(hardwareMap, telemetry);
        deliveryGrippers.init(hardwareMap, telemetry);
        follower = new Follower(hardwareMap);
        poseUpdater = new PoseUpdater(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(80);
        poseUpdater.setStartingPose(startPose);

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(highBar)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());
        scorePreloadChain = follower.pathBuilder()
                .addPath(scorePreload)
                .addTemporalCallback(0, ()->{

                })
                .build();



        waitForStart();
        stepTimer.reset();
        while(opModeIsActive()) {
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
            if(step == 1){
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                slides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BAR_AUTO,1 );
                slides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BAR_AUTO, 1);
                deliveryAxon.setPosition(constants.DELIVERY_HIGH_BAR);
                if(stepTimer.seconds() > 2) {
                    if (!trajFollowed) {
                        follower.followPath(scorePreloadChain, true);
                        trajFollowed = true;
                        stepTimer.reset();
                    }
                    if (!follower.isBusy()) {
                        if (stepTimer.seconds() > 1 && stepTimer.seconds() < 3) {
                            deliveryAxon.setPosition(constants.DELIVERY_HOOK);
                        }
                        if(stepTimer.seconds() > 3 && stepTimer.seconds() < 5) {
                            deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                            stepTimer.reset();
                            step = 2;
                            trajFollowed = false;
                        }
                    }
                    else {
                        stepTimer.reset();
                    }
                }
            }
            if(step == 2){
                while (stepTimer.seconds() < 2){
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                    if(stepTimer.seconds() > 1){
                        deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                    }
                }
                step = 3;
                stepTimer.reset();
            }
        }

    }

}
