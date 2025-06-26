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
import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotV2;
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

@Autonomous(name = "Specimen Auto (V3.0)", group = "Coach")
public class SpecimenAutoV3 extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer;
    public RobotV2 robot = new RobotV2();
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
        READY_FOR_PICKUP,
        PICK_UP_SPECIMEN_1,
        DELIVERY_POSITION_1,
        HANG_SPECIMEN_1,
        PICK_UP_SPECIMEN_2,
        DELIVERY_POSITION_2,
        HANG_SPECIMEN_2,
        PICK_UP_SPECIMEN_3,
        DELIVERY_POSITION_3,
        HANG_SPECIMEN_3,
        IDLE
    }
    public PathState pathState = PathState.START;
    public ActionState actionState = ActionState.DELIVERY_POSITION;
    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 63, Math.toRadians(180));
    private final Pose tosubmersiblePose = new Pose(36.5, 72, Math.toRadians(180));
    private final Pose toSubmersibleTarget = new Pose(70.5, 72, Math.toRadians(180));
    private final Pose strafeoutPose = new Pose(30, 36, Math.toRadians(180));
    public final Pose strafeBehind1 = new Pose (60, 36, Math.toRadians(180));
    private final Pose strafebehindsample1Pose = new Pose(60, 25, Math.toRadians(180));
    private final Pose pushsample1Pose = new Pose(13, 25, Math.toRadians(180));
    private final Pose straightBack = new Pose(20, 25, Math.toRadians(180));
    private final Pose strafebehindsample2Pose = new Pose(60, 15, Math.toRadians(180));
    private final Pose pushsample2Pose = new Pose(13, 15, Math.toRadians(180));
    private final Pose positionspecimenpickupPose = new Pose(36, 15, Math.toRadians(180));
    private final Pose behindSubmersible = new Pose(25, 76, Math.toRadians(180));
    private final Pose toBar = new Pose(36.5, 76, Math.toRadians(180));
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
    public Path scorePreload, pushSample01, strafeOut, strafeBehindSample1, releaseAndBack, inFrontOfSub, deliverSample2, strafeBehind01, strafeBehind2, pushSample2, positionSpecimenPickup, straightBackPath;
    public PathChain strafeout, strafebehindsample1, pushSample1, straightBackChain, backwardsFromSample1, deliverSpecimen2, pickUpSpecimen1, toSubmersible1, forwardToSubmersible1, backwardsFromSubmersible1, pickUpSpecimen2, strafeToSubmersible2, forwardToSubmersible2, backwardsFromSubmersible2, pickUpSpecimen3, strafeToSubmersible3, forwardToSubmersible4, backwardsFromSubmersible3, strafeToPark;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {


        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(toSubmersibleTarget)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), toSubmersibleTarget.getHeading());
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

        positionSpecimenPickup = new Path(new BezierLine(new Point(pushsample1Pose), new Point(positionspecimenpickupPose)));
        positionSpecimenPickup.setLinearHeadingInterpolation(pushsample1Pose.getHeading(), positionspecimenpickupPose.getHeading());

        pushSample1 = follower.pathBuilder()
                .addPath(strafeOut)
                .addPath(strafeBehind01)
                .addPath(strafeBehindSample1)
                .addPath(pushSample01)
                .build();

        straightBackPath = new Path(new BezierLine(new Point(pushsample1Pose), new Point(straightBack)));
        straightBackPath.setLinearHeadingInterpolation(pushsample1Pose.getHeading(), straightBack.getHeading());
        inFrontOfSub = new Path(new BezierLine(new Point(straightBack), new Point(behindSubmersible)));
        inFrontOfSub.setLinearHeadingInterpolation(straightBack.getHeading(), behindSubmersible.getHeading());
        deliverSample2 = new Path(new BezierLine(new Point(behindSubmersible), new Point(toBar)));
        deliverSample2.setLinearHeadingInterpolation(behindSubmersible.getHeading(), toBar.getHeading());
        toSubmersible1 = follower.pathBuilder()
                .addPath(inFrontOfSub)
                .addPath(deliverSample2)
                .setLinearHeadingInterpolation(tosubmersiblePose.getHeading(), tosubmersiblePose.getHeading())
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousActionUpdate(){
        switch (actionState){
            case DELIVERY_POSITION:
                robot.setSystemState(RobotV2.SystemStates.DELIVER_HIGH_BAR);
                break;
            case HANG_PRELOAD:
                robot.setSystemState(RobotV2.SystemStates.CLIP_HIGH_BAR);
                break;
            case TRAVEL:
                robot.setSystemState(RobotV2.SystemStates.START);
                break;
            case READY_FOR_PICKUP:
                robot.setSystemState(RobotV2.SystemStates.LOW_HOVER);
                if(actionTimer.seconds() > 4){
                    setActionState(ActionState.PICK_UP_SPECIMEN_1);
                }
                break;
            case PICK_UP_SPECIMEN_1:
                if(actionTimer.seconds() < 1 && actionTimer.seconds() > 0){
                   robot.setSystemState(RobotV2.SystemStates.DOWN_ON_SAMPLE);
                }
                if(actionTimer.seconds() > 1 && actionTimer.seconds() < 2.75){
                    robot.setSystemState(RobotV2.SystemStates.DOWN_ON_SAMPLE_GRAB);
                }
                if(actionTimer.seconds() > 5.75){
                    setPathState(PathState.TO_SUBMERSIBLE_1);
                    setActionState(ActionState.DELIVERY_POSITION);
                }
                break;

        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case GO_TO_SUBMERSIBLE:
                if(!follower.isBusy()) {
                    follower.followPath(scorePreload);
                }
                if(follower.getPose().getX() >= tosubmersiblePose.getX()) {
                    setPathState(PathState.RELEASE_AND_BACK);
                    actionTimer.reset();
                    actionStateSet = false;
                }
                break;
            case RELEASE_AND_BACK:
                    if(!actionStateSet) {
                        follower.setMaxPower(0.7);
                        setActionState(ActionState.HANG_PRELOAD);
                        actionStateSet = true;
                    }
                    if (actionTimer.seconds() > 1) {
                        setPathState(PathState.PUSH_SAMPLE1);
                        actionStateSet = false;
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
                    follower.followPath(straightBackPath);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    setPathState(PathState.PUSH_SAMPLE2);
                }
                break;
            case PUSH_SAMPLE2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(PathState.POSITION_SPECIMEN_PICKUP);
                }
                break;
            case POSITION_SPECIMEN_PICKUP:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    follower.followPath(positionSpecimenPickup);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    setPathState(PathState.PICK_UP_SPECIMEN_1);
                    actionStateSet = false;
                    actionTimer.reset();
                    pathTimer.reset();
                }
                break;
            case PICK_UP_SPECIMEN_1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */
                    if(!actionStateSet) {
                        setActionState(ActionState.READY_FOR_PICKUP);
                        actionStateSet = true;
                    }
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */;
                }
                break;
            case TO_SUBMERSIBLE_1:
                if(!follower.isBusy()) {
                    follower.followPath(toSubmersible1);
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
                if(!follower.isBusy()){
                    setActionState(ActionState.HANG_PRELOAD);
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
        robot.systemStatesUpdate();
        autonomousActionUpdate();
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addLine("Action state: " + actionState.toString());
        telemetry.addLine("Action state timer: " + actionTimer.seconds());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        opmodeTimer.reset();
        robot.init(hardwareMap, telemetry, RobotV2.Mode.AUTO, gamepad1);
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

