//Work in progress, thanks to Monkey Machina 26433 for the original file

package org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.examples;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotV2;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.FConstantsLeft;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.FConstantsRight;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.LConstantsRight;

@Autonomous
public class SpecimenAutoV4 extends OpMode {

    private RobotV2 robot;
    private GamepadEx controller;
    private DashboardPoseTracker dashboardPoseTracker;
    private Follower follower;

    private enum AutoState {
        start,
        specDepoOne,
        intakingSpecOne,
        specDepoTwo,
        samplePushOne,
        samplePushTwo,
        samplePushThree,
        intakingSpecTwo,
        depositingSpecThree,
        intakingSpecThree,
        depositingSpecFour,
        intakingSpecFour,
        depositingSpecFive,
        park,
        done;
    }

    private AutoState autoState = AutoState.start;

    private enum SpecIntakingStatus {
        aligning,
        intaking
    }

    private SpecIntakingStatus specIntakingStatus = SpecIntakingStatus.aligning;

    private enum SpecDepoStatus {
        driving,
        releasing;
    }

    private SpecDepoStatus specDepoStatus = SpecDepoStatus.driving;

    private PathChain specDepoOnePC, specIntakeOnePC, specDepoTwoPC, samplePushOnePC, samplePushTwoPC, samplePushThreePC, specIntakeTwoPC, specDepoThreePC, specIntakeThreePC, specDepoFourPC, specIntakeFourPC, specDepoFivePC, parkPC;

    @Override
    public void init() {
        // Pedro & Path Setup
        SpecimenAutoPaths.build();
        controller = new GamepadEx(gamepad1);
        robot = new RobotV2();
        robot.init(hardwareMap, telemetry, RobotV2.Mode.AUTO, gamepad1);
        Constants.setConstants(FConstantsLeft.class, LConstantsRight.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(SpecimenAutoPaths.startPose);
        dashboardPoseTracker = new DashboardPoseTracker(follower.poseUpdater);
        Drawing.drawRobot(follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
        buildPaths();

        try {
            Thread.sleep(350);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.deliveryGrippers.setPosition(robot.constants.DELIVERY_GRIPPERS_CLOSE);

        // Robot Setup

    }

    @Override
    public void init_loop() {
        controller.readButtons();
        robot.deliveryGrippers.setPosition(robot.constants.DELIVERY_GRIPPERS_CLOSE);
        robot.update();
    }

    @Override
    public void start(){
        robot.resetStateTimer();
        robot.setSystemState(RobotV2.SystemStates.DELIVER_HIGH_BAR);
        robot.grippers.setPosition(robot.constants.GRIPPERS_CLOSE);
    }

    @Override
    public void loop() {
        autoStateUpdate();
        telemetry.addLine(autoState.name());
        robot.systemStatesUpdate();
        controller.readButtons();
        dashboardPoseTracker.update();
        follower.update();
        if (follower.getCurrentPath() != null) {
            Drawing.drawPath(follower.getCurrentPath(), "#808080");
        }

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(follower.poseUpdater.getPose(), "#4CAF50");

    }

    public void autoStateUpdate() {
        logValues();
        switch (autoState) {
            case start:
                //if (robot.deposit.claw.currentState == Claw.State.Closed) {
                //robot.setDepositDesiredState(Deposit.State.specDeposit);
                if(robot.stateTimer.seconds() > 4) {
                    follower.followPath(specDepoOnePC, false);
                    autoState = AutoState.specDepoOne;
                }
                //}
                break;

            case specDepoOne:
                depositSpec(specIntakeOnePC, AutoState.intakingSpecOne);
                break;

            case intakingSpecOne:
                intakeSpec(specDepoTwoPC, AutoState.specDepoTwo, 0.5, 0.3);
                break;

            case specDepoTwo:
                depositSpec(samplePushOnePC, AutoState.samplePushOne);
                break;

            case samplePushOne:
                follower.setCentripetalScaling(0.0014);
                follower.setMaxPower(0.9);
                if (!follower.isBusy() || follower.getPose().getX() <= 15) {
                    follower.followPath(samplePushTwoPC);
                    autoState = AutoState.samplePushTwo;
                }

                break;

            case samplePushTwo:
                //follower.setCentripetalScaling(0.0014);
                follower.setMaxPower(0.9);
                if (follower.getPose().getX() <= 15 && follower.getCurrentTValue() >= 0.5) {
                    follower.followPath(samplePushThreePC, true);
                    autoState = AutoState.samplePushThree;
                }
                break;

            case samplePushThree:
                //follower.setCentripetalScaling(0.0014);
                follower.setMaxPower(0.9);
                if (follower.getPose().getX() <= 15 && follower.getCurrentTValue() >= 0.5) {
                    follower.followPath(specIntakeTwoPC, true);
                    autoState = AutoState.intakingSpecTwo;
                    follower.setCentripetalScaling(0.0012);
                    follower.setMaxPower(1.0);
                }
                break;

            case intakingSpecTwo:
                intakeSpec(specDepoThreePC, AutoState.depositingSpecThree, 0.7, 0.3);
                break;

            case depositingSpecThree:
                depositSpec(specIntakeThreePC, AutoState.intakingSpecThree);
                break;

            case intakingSpecThree:
                intakeSpec(specDepoFourPC, AutoState.depositingSpecFour, 0.5, 0.3);
                break;

            case depositingSpecFour:
                depositSpec(specIntakeFourPC, AutoState.intakingSpecFour);
                break;

            case intakingSpecFour:
                intakeSpec(specDepoFivePC, AutoState.depositingSpecFive, 0.5, 0.3);
                break;

            case depositingSpecFive:
                depositSpec(parkPC, AutoState.park);
                break;

            case park:
                if (!follower.isBusy()) {
                    autoState = AutoState.done;
                }
        }

    }

    private void buildPaths() {
        specDepoOnePC = follower
                .pathBuilder().addPath(SpecimenAutoPaths.specDepoOne)
                .setConstantHeadingInterpolation(SpecimenAutoPaths.specDepoOnePose.getHeading())
                .setPathEndTimeoutConstraint(250.0)
                .build();

        specIntakeOnePC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.specIntakeOne)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.specIntakeOnePose.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .build();

        specDepoTwoPC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.specDepoTwo)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.specDepoTwoPose.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .build();

        samplePushOnePC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.samplePushOne)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.pushPoseOne.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .setZeroPowerAccelerationMultiplier(6)
                        .build();

        samplePushTwoPC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.samplePushTwo)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.pushPoseTwo.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .setZeroPowerAccelerationMultiplier(6)
                        .build();

        samplePushThreePC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.samplePushThreeCurve)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.pushPoseThree1.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .setZeroPowerAccelerationMultiplier(6)
                        .addPath(SpecimenAutoPaths.samplePushThreeLine)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.pushPoseThree2.getHeading())
                        .build();

        specIntakeTwoPC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.specIntakeTwo)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.specIntakeTwoPose.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .build();

        specDepoThreePC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.specDepoThree)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.specDepoThreePose.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .build();

        specIntakeThreePC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.specIntakeThree)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.specIntakeThreePose.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .build();

        specDepoFourPC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.specDepoFour)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.specDepoFourPose.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .build();

        specIntakeFourPC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.specIntakeFour)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.specIntakeFourPose.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .build();

        specDepoFivePC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.specDepoFive)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.specDepoFivePose.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .build();

        parkPC =
                follower
                        .pathBuilder().addPath(SpecimenAutoPaths.park)
                        .setConstantHeadingInterpolation(SpecimenAutoPaths.parkPose.getHeading())
                        .setPathEndTimeoutConstraint(250.0)
                        .build();
    }

    private void intakeSpec(PathChain nextPath, AutoState nextAutoState, double slowTValue, double slowPower) {

        // If we are close to the end of the path, slow down to grab spec, only if in aligning state
        if (follower.getCurrentTValue() >= slowTValue && specIntakingStatus == SpecIntakingStatus.aligning) {
            follower.setMaxPower(slowPower);
        }


        follower.followPath(nextPath);
        autoState = nextAutoState;
        specIntakingStatus = SpecIntakingStatus.aligning;

    }

    private void depositSpec(PathChain nextPath, AutoState nextAutoState) {
        follower.update();
        //follower.setCentripetalScaling(0.002);
        // If Vx meets velocity constraint and the path did not just start (t>=0.1) and specDepoStatus is driving, move to releasing status
        if (follower.getPose().getX() >= SpecimenAutoPaths.specDepoOnePoseTarget.getX() && specDepoStatus == SpecDepoStatus.driving) {
            // if ((!follower.isBusy()) && specDepoStatus == SpecDepoStatus.driving) {
            robot.setSystemState(RobotV2.SystemStates.CLIP_HIGH_BAR);
            follower.holdPoint(SpecimenAutoPaths.specDepoOnePoseTarget);
            follower.breakFollowing();
            specDepoStatus = SpecDepoStatus.releasing;
        } else {
            robot.setSystemState(RobotV2.SystemStates.DELIVER_HIGH_BAR);
            robot.systemStatesUpdate();
        }

        // If the claw is open and specDepoStatus is releasing, move to next path.
        if (specDepoStatus == SpecDepoStatus.releasing && robot.stateTimer.seconds() > 4 && !follower.isBusy()) {
            follower.followPath(nextPath);
            autoState = nextAutoState;
            specDepoStatus = SpecDepoStatus.driving;
            //follower.setCentripetalScaling(0.0012);
        }
    }

    public void logValues(){
        telemetry.addLine("x: " + follower.getPose().getX());
        telemetry.addLine("y: " + follower.getPose().getY());
        telemetry.addLine("Follower is busy: " +  follower.isBusy());
        telemetry.addLine("T value: " + follower.getCurrentTValue());
        telemetry.addLine("Velocity constraints met: " + (Math.abs(follower.getVelocity().getXComponent()) <= 1));
    }
}
