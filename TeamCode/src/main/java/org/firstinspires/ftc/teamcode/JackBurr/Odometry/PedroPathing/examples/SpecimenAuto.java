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

import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.LConstants;

/**
 *This is a 1 plus 2 specimen delivery and park
 */

@Autonomous(name = "Specimen Auto", group = "Coach")
public class SpecimenAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /*State machine for pathing */
    enum PathState{
        GO_TO_SUBMERSIBLE,
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
    enum ActionState{
        DELIVERY_POSITION,
        HANG_PRELOAD,
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
    PathState pathState = PathState.GO_TO_SUBMERSIBLE;
    ActionState actionState = ActionState.DELIVERY_POSITION;
    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 63, Math.toRadians(180));
    private final Pose tosubmersiblePose = new Pose(35, 63, Math.toRadians(180));
    private final Pose strafeoutPose = new Pose(31, 45, Math.toRadians(180));
    private final Pose strafebehindsample1Pose = new Pose(61, 27, Math.toRadians(180));
    private final Pose pushsample1Pose = new Pose(13, 27, Math.toRadians(180));
    private final Pose backwarsdfromsample1Pose = new Pose(67, 24, Math.toRadians(180));
    private final Pose strafebehindsample2Pose = new Pose(67, 15.5, Math.toRadians(180));
    private final Pose pushsample2Pose = new Pose(13, 15.5, Math.toRadians(180));
    private final Pose positionspecimenpickupPose = new Pose(30, 35, Math.toRadians(230));
    private final Pose pickupspecimen1Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose tosubmersible1Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose forwardtosubmersible1Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose backwarsdfromsubmersible1Pose = new Pose(49, 135, Math.toRadians(0));
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
    private Path scorePreload;
    private PathChain strafeout, strafebehindsample1;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {


        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(tosubmersiblePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), tosubmersiblePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        strafeout = follower.pathBuilder()
                .addPath(new BezierLine(new Point(tosubmersiblePose), new Point(strafeoutPose)))
                .setLinearHeadingInterpolation(tosubmersiblePose.getHeading(), strafeoutPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        strafebehindsample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(strafeoutPose), new Point(strafebehindsample1Pose)))
                .setLinearHeadingInterpolation(strafeoutPose.getHeading(), strafebehindsample1Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
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

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case GO_TO_SUBMERSIBLE:
                follower.followPath(scorePreload);
                setPathState(pathState.STRAFE_OUT);
                break;
            case STRAFE_OUT:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(strafeout,true);
                    setPathState(pathState.STRAFE_BEHIND_SAMPLE1);
                }
                break;
            case STRAFE_BEHIND_SAMPLE1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(strafebehindsample1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
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
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

