package org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Sample Auto", group = "Coach")
public class BucketAuto extends OpMode {

    public DeliveryGrippersV1 deliveryGrippers = new DeliveryGrippersV1();
    public DeliveryAxonV1 deliveryAxon = new DeliveryAxonV1();
    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public RobotConstantsV1 constants = new RobotConstantsV1();
    public GrippersV1 grippers = new GrippersV1();
    public IntakeSlidesV1 intakeSlides = new IntakeSlidesV1();
    public DifferentialV2 diffV2 = new DifferentialV2();
    public WristAxonV1 wrist = new WristAxonV1();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public ElapsedTime stateTimer = new ElapsedTime();
    public boolean pathFollowed = false;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(20, 124, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(24, 121, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(24, 131.5, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(24, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(80, 96, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(80, 120,Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain scorePreloadChain, parkChain, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreloadChain = follower.pathBuilder()
                .addPath(scorePreload)
                .addTemporalCallback(0, ()->{
                    slides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BASKET, 1);
                    slides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BASKET, 1);
                })
                .addTemporalCallback(3000, ()->{
                    deliveryAxon.setPosition(constants.DELIVERY_UP);
                })
                .build();
        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .addTemporalCallback(0, ()->{
                    slides.runLeftSlideToPosition(0, 1);
                    slides.runRightSlideToPosition(0, 1);
                    intakeSlides.intakeOut();
                    grippers.setPosition(constants.GRIPPERS_OPEN);
                    diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_HOVER);
                    diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_HOVER);
                })
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addTemporalCallback(0, ()->{
                    slides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BASKET,1);
                    slides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BASKET,1);
                })
                .addTemporalCallback(50, ()->{
                    deliveryAxon.setPosition(constants.DELIVERY_UP);
                })
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addTemporalCallback(0, ()->{
                    slides.runLeftSlideToPosition(0, 1);
                    slides.runRightSlideToPosition(0, 1);
                    intakeSlides.intakeOut();
                    grippers.setPosition(constants.GRIPPERS_OPEN);
                    diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_HOVER);
                    diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_HOVER);
                })
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addTemporalCallback(0, ()->{
                    slides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BASKET,1);
                    slides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BASKET,1);
                })
                .addTemporalCallback(50, ()->{
                    deliveryAxon.setPosition(constants.DELIVERY_UP);
                })
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
        parkChain = follower.pathBuilder()
                .addPath(park)
                .addTemporalCallback(0, ()->{
                    slides.runLeftSlideToPosition(0, 1);
                    slides.runRightSlideToPosition(0, 1);
                })
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                wrist.setPosition(constants.WRIST_CENTER);
                if(!pathFollowed){
                    follower.followPath(scorePreloadChain);
                    stateTimer.reset();
                    pathFollowed = true;
                }
                if(pathFollowed && !follower.isBusy()) {
                    if (stateTimer.seconds() < 3 && stateTimer.seconds() > 2.25){
                        deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                    }
                    if (stateTimer.seconds() < 4.25 && stateTimer.seconds() > 3){
                        deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                    }
                    if(stateTimer.seconds() > 4.25) {
                        setPathState(1);
                        stateTimer.reset();
                        pathFollowed = false;
                    }
                }
                else if(pathFollowed){
                    if((Math.abs(slides.getLeftSlidePosition()) > Math.abs(constants.LEFT_SLIDE_HIGH_BASKET / 2)) && (Math.abs(slides.getRightSlidePosition()) > Math.abs(constants.RIGHT_SLIDE_HIGH_BASKET / 2))) {
                        deliveryAxon.setPosition(constants.DELIVERY_UP);
                    }
                }
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                wrist.setPosition(constants.WRIST_CENTER);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(!pathFollowed) {
                        follower.followPath(grabPickup1, true);
                        pathFollowed = true;
                    }
                    if(pathFollowed && stateTimer.seconds() < 2){
                        diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_PICKUP);
                        diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_PICKUP);
                    }
                    else if(pathFollowed && stateTimer.seconds() > 2 && stateTimer.seconds() < 2.75){
                        grippers.setPosition(constants.GRIPPERS_GRAB);
                    }
                    else if(pathFollowed && stateTimer.seconds() > 2.75 && stateTimer.seconds() < 4.75){
                        if(stateTimer.seconds() > 4){
                            intakeSlides.intakeAllTheWayIn();
                        }
                        diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                        diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                    }
                    else if(pathFollowed && stateTimer.seconds() > 4.75 & stateTimer.seconds() < 6){
                        if(stateTimer.seconds() > 5.15){
                            grippers.setPosition(constants.GRIPPERS_OPEN);
                        }
                        else {
                            deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                        }
                    }
                    else if(stateTimer.seconds() > 6) {
                        setPathState(2);
                        stateTimer.reset();
                        pathFollowed = false;
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    wrist.setPosition(constants.WRIST_CENTER);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(!pathFollowed) {
                        follower.followPath(scorePickup1, true);
                        stateTimer.reset();
                        pathFollowed = true;
                    }
                    if(pathFollowed && !follower.isBusy()){
                        if(stateTimer.seconds() < 2){
                            deliveryAxon.setPosition(constants.DELIVERY_UP);
                        }
                        if (stateTimer.seconds() < 2.75 && stateTimer.seconds() > 2){
                            deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                        }
                        if (stateTimer.seconds() < 3.75 && stateTimer.seconds() > 2.75){
                            deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                        }
                        if(stateTimer.seconds() > 3.75) {
                            deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                            setPathState(3);
                            stateTimer.reset();
                            pathFollowed = false;
                        }
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    /* Score Preload */
                    deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                    wrist.setPosition(constants.WRIST_CENTER);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(!pathFollowed) {
                        follower.followPath(grabPickup2, true);
                        pathFollowed = true;
                    }
                    if(pathFollowed && stateTimer.seconds() < 2){
                        diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_PICKUP);
                        diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_PICKUP);
                    }
                    else if(pathFollowed && stateTimer.seconds() > 2 && stateTimer.seconds() < 3){
                        grippers.setPosition(constants.GRIPPERS_GRAB);
                    }
                    else if(pathFollowed && stateTimer.seconds() > 3 && stateTimer.seconds() < 4.75){
                        if(stateTimer.seconds() > 4){
                            intakeSlides.intakeAllTheWayIn();
                        }
                        diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                        diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                    }
                    else if(pathFollowed && stateTimer.seconds() > 4.75 & stateTimer.seconds() < 6){
                        if(stateTimer.seconds() > 5.15){
                            grippers.setPosition(constants.GRIPPERS_OPEN);
                        }
                        else {
                            deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                        }
                    }
                    else if(stateTimer.seconds() > 6) {
                        setPathState(4);
                        stateTimer.reset();
                        pathFollowed = false;
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    wrist.setPosition(constants.WRIST_CENTER);
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if (!pathFollowed) {
                        follower.followPath(scorePickup2, true);
                        stateTimer.reset();
                        pathFollowed = true;
                    }
                    if (pathFollowed && !follower.isBusy()) {
                        if (stateTimer.seconds() < 2) {
                            deliveryAxon.setPosition(constants.DELIVERY_UP);
                        }
                        if (stateTimer.seconds() < 3 && stateTimer.seconds() > 2) {
                            deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                        }
                        if (stateTimer.seconds() < 4.25 && stateTimer.seconds() > 3) {
                            deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                        }
                        if (stateTimer.seconds() > 4.25 && stateTimer.seconds() < 6) {
                            slides.runLeftSlideToPosition(0, 1);
                            slides.runRightSlideToPosition(0, 1);
                        }
                        if (stateTimer.seconds() > 6) {
                            setPathState(5);
                            stateTimer.reset();
                            pathFollowed = false;
                        }
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    wrist.setPosition(constants.WRIST_CENTER);
                    diffV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                    diffV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                    intakeSlides.intakeAllTheWayIn();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(parkChain,true);
                    deliveryAxon.setPosition(constants.DELIVERY_LEVEL_ONE_ASCENT);
                    setPathState(6);
                }
                break;
            case 6:
                deliveryAxon.setPosition(constants.DELIVERY_LEVEL_ONE_ASCENT);
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Left slide: ", slides.getLeftSlidePosition());
        telemetry.addData("Right slide: ", slides.getRightSlidePosition());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        stateTimer.reset();
        deliveryGrippers.init(hardwareMap);
        deliveryAxon.init(hardwareMap);
        diffV2.init(hardwareMap);
        grippers.init(hardwareMap);
        slides.init(hardwareMap);
        intakeSlides.init(hardwareMap);
        intakeSlides.resetSlides();
        slides.resetSlides();
        wrist.init(hardwareMap);
        Constants.setConstants(FConstantsLeft.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_GRAB);
        deliveryAxon.setPosition(constants.DELIVERY_GRAB);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        stateTimer.reset();
        pathTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

