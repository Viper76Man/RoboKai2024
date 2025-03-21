package org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
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
 *This is a 1 plus 2 specimen delivery and park
 */

@Autonomous(name = "Specimen Auto", group = "Coach")
public class SpecimenAuto extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer;

    public IntakeSlidesV1 intakeSlides = new IntakeSlidesV1();
    public DifferentialV2 diffV2 = new DifferentialV2();
    public GrippersV1 grippers = new GrippersV1();
    public WristAxonV1 wrist = new WristAxonV1();
    public DeliveryAxonV1 deliveryAxon = new DeliveryAxonV1();
    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public DeliveryGrippersV1 deliveryGrippers = new DeliveryGrippersV1();
    public RobotConstantsV1 constants = new RobotConstantsV1();
    public boolean pathStateSet = false;
    public boolean actionStateSet = false;
    public boolean pathStateSet2 = false;
    /*State machine for pathing */
    public enum PathState{
        START,
        GO_TO_SUBMERSIBLE,
        RELEASE_AND_BACK,
        STRAFE_OUT,
        STRAFE_BEHIND_SAMPLE1,
        PUSH_SAMPLE1,
        BACKWARDS_FROM_SAMPLE1,
        STRAFE_BEHIND_SAMPLE2,
        PUSH_SAMPLE2,
        POSITION_SPECIMEN_PICKUP,
        PICK_UP_SPECIMEN_1,
        TO_SUBMERSIBLE_1,
        FORWARD_TO_SUBMERSIBLE_1,
        BACKWARDS_FROM_SUBMERSIBLE_1,
        PICK_UP_SPECIMEN_2,
        STRAFE_TO_SUBMERSIBLE_2,
        FORWARD_TO_SUBMERSIBLE_2,
        BACKWARDS_FROM_SUBMERSIBLE_2,
        PICK_UP_SPECIMEN_3,
        STRAFE_TO_SUBMERSIBLE_3,
        FORWARD_TO_SUBMERSIBLE_3,
        BACKWARDS_FROM_SUBMERSIBLE_3,
        STRAFE_TO_PARK,
        DONE
    }
    /*State machine for actions*/
    public enum ActionState{
        DELIVERY_POSITION,
        HANG_PRELOAD,
        TRAVEL,
        PICK_UP_SPECIMEN_1,
        DELIVERY_POSITION_1,
        HANG_SPECIMEN_1,
        PICK_UP_SPECIMEN_2,
        DELIVERY_POSITION_2,
        HANG_SPECIMEN_2,
        PICK_UP_SPECIMEN_3,
        DELIVERY_POSITION_3,
        HANG_SPECIMEN_3,

    }
    public PathState pathState = PathState.START;
    public ActionState actionState = ActionState.DELIVERY_POSITION;
    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 63, Math.toRadians(180));
    private final Pose tosubmersiblePose = new Pose(36.5, 72, Math.toRadians(180));
    private final Pose strafeoutPose = new Pose(30, 36, Math.toRadians(180));
    public final Pose strafeBehind1 = new Pose (60, 36, Math.toRadians(180));
    private final Pose strafebehindsample1Pose = new Pose(60, 25, Math.toRadians(180));
    private final Pose pushsample1Pose = new Pose(13, 25, Math.toRadians(180));
    private final Pose straightBack = new Pose(60, 25, Math.toRadians(180));
    private final Pose strafebehindsample2Pose = new Pose(60, 15, Math.toRadians(180));
    private final Pose pushsample2Pose = new Pose(13, 15, Math.toRadians(180));
    private final Pose positionspecimenpickupPose = new Pose(35.5, 15, Math.toRadians(180));
    private final Pose pickupspecimen1Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose tosubmersible1Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose forwardtosubmersible1Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose backwardsfromsubmersible1Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose pickupspecimen2Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose tosubmersible2Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose forwardtosubmersible2Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose backwardsfromsubmersible2Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose pickupspecimen3Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose tosubmersible3Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose forwardtosubmersible3Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose backwardfromsubmersible3Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose strafetoparkPose = new Pose(49, 135, Math.toRadians(0));



    /* These are our Paths and PathChains that we will define in buildPaths() */
    public Path scorePreload, pushSample01, strafeOut, strafeBehindSample1, releaseAndBack, strafeBehind01, strafeBehind2;
    public PathChain strafeout, strafebehindsample1, pushSample1, straightBackChain, backwardsFromSample1, pushSample2, positionSpecimenPickup, pickUpSpecimen1, toSubmersible1, forwardToSubmersible1, backwardsFromSubmersible1, pickUpSpecimen2, strafeToSubmersible2, forwardToSubmersible2, backwardsFromSubmersible2, pickUpSpecimen3, strafeToSubmersible3, forwardToSubmersible4, backwardsFromSubmersible3, strafeToPark;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {


        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(tosubmersiblePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), tosubmersiblePose.getHeading());
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        strafeOut = new Path(new BezierLine(new Point(tosubmersiblePose), new Point(strafeoutPose)));
        strafeOut.setLinearHeadingInterpolation(tosubmersiblePose.getHeading(), strafeoutPose.getHeading());
        strafeBehind01 = new Path(new BezierLine(new Point(strafeoutPose), new Point(strafeBehind1)));
        strafeBehind01.setLinearHeadingInterpolation(strafeoutPose.getHeading(), strafeBehind1.getHeading());
        strafeBehindSample1 = new Path(new BezierLine(new Point(strafeBehind1), new Point(strafebehindsample1Pose)));
        strafeBehindSample1.setLinearHeadingInterpolation(strafeBehind1.getHeading(), strafebehindsample1Pose.getHeading());
        pushSample01 = new Path(new BezierLine(new Point(strafebehindsample1Pose), new Point(pushsample1Pose)));
        pushSample01.setLinearHeadingInterpolation(strafebehindsample1Pose.getHeading(), pushsample1Pose.getHeading());
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushSample1 = follower.pathBuilder()
                .addPath(strafeOut)
                .addPath(strafeBehind01)
                .addPath(strafeBehindSample1)
                .addPath(pushSample01)
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        straightBackChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushsample1Pose), new Point(straightBack)))
                .setLinearHeadingInterpolation(pushsample1Pose.getHeading(), straightBack.getHeading())
                .build();

        strafeBehind2 = new Path(new BezierLine(new Point(straightBack), new Point(strafebehindsample2Pose)));
        strafeBehind2.setLinearHeadingInterpolation(straightBack.getHeading(), strafebehindsample2Pose.getHeading());

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushSample2 = follower.pathBuilder()
                .addPath(strafeBehind2)
                .addPath(new BezierLine(new Point(strafebehindsample2Pose), new Point(pushsample2Pose)))
                .setLinearHeadingInterpolation(strafebehindsample2Pose.getHeading(), pushsample2Pose.getHeading())
                .build();

        positionSpecimenPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushsample2Pose), new Point(positionspecimenpickupPose)))
                .setLinearHeadingInterpolation(pushsample2Pose.getHeading(), positionspecimenpickupPose.getHeading())
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousActionUpdate(){
        switch (actionState){
            case DELIVERY_POSITION:
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                if(actionTimer.seconds() < 4 && actionTimer.milliseconds() > 200) {
                    deliveryAxon.setPosition(constants.DELIVERY_CLIP);
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                    slides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BAR_AUTO, 1);
                    slides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BAR_AUTO, 1);
                }
                if(actionTimer.seconds() > 1) {
                    if(!pathStateSet2) {
                        setPathState(PathState.GO_TO_SUBMERSIBLE);
                        pathStateSet2 = true;
                    }
                }
                break;
            case HANG_PRELOAD:
                //slides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BAR_CLIP_AUTO, 1);
                //slides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BAR_CLIP_AUTO, 1);
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                break;
            case TRAVEL:
                slides.runLeftSlideToPosition(0, 1);
                slides.runRightSlideToPosition(0, 1);
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                break;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case GO_TO_SUBMERSIBLE:
                if(!follower.isBusy()){
                    follower.followPath(scorePreload);
                    setPathState(PathState.RELEASE_AND_BACK);
                    actionTimer.reset();
                    actionStateSet = false;
                }
                break;
            case RELEASE_AND_BACK:
                if(!follower.isBusy()) {
                    if(!actionStateSet) {
                        follower.setMaxPower(0.7);
                        setActionState(ActionState.HANG_PRELOAD);
                        actionStateSet = true;
                    }
                    if (actionTimer.seconds() > 1) {
                        setPathState(PathState.PUSH_SAMPLE1);
                        actionStateSet = false;
                    }
                }
                else {
                    actionTimer.reset();
                }
                break;
            case PUSH_SAMPLE1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    if(!actionStateSet){
                        setActionState(ActionState.TRAVEL);
                        actionStateSet = true;
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pushSample1,true);
                    setPathState(PathState.STRAFE_BEHIND_SAMPLE2);
                }
                break;
            case STRAFE_BEHIND_SAMPLE2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(straightBackChain,true);
                    setPathState(PathState.PUSH_SAMPLE2);
                }
                break;
            case PUSH_SAMPLE2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pushSample2, true);
                    setPathState(PathState.DONE);
                }
                break;
            case POSITION_SPECIMEN_PICKUP:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(positionSpecimenPickup,true);
                    setPathState(PathState.DONE);
                }
                break;
            case PICK_UP_SPECIMEN_1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */
                    if(pathTimer.seconds() > 3){
                        setPathState(PathState.DONE);
                    }
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                }
                else {
                    pathTimer.reset();
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(PathState pState) {
        pathState = pState;
        pathTimer.reset();
    }

    public void setActionState(ActionState aState) {
        actionState = aState;
        actionTimer.reset();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        opmodeTimer.reset();
        deliveryAxon.init(hardwareMap);
        deliveryGrippers.init(hardwareMap);
        slides.init(hardwareMap);
        intakeSlides.init(hardwareMap);
        intakeSlides.resetSlides();
        slides.resetSlides();
        diffV2.init(hardwareMap);
        grippers.init(hardwareMap);
        wrist.init(hardwareMap);
        Constants.setConstants(FConstantsLeft.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.reset();
        actionTimer.reset();
        pathTimer.reset();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

