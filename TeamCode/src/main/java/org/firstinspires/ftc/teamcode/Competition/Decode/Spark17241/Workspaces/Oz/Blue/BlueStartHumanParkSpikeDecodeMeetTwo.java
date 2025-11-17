package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.AutoMain_NewAndrew;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;

//@Disabled
@Autonomous(name = "Blue:Start Human Park Spike  Meet Two ", group = "Drive")
public class BlueStartHumanParkSpikeDecodeMeetTwo extends AutoMain_NewAndrew {

    /**  Pedro Pathing Variables, Poses, Paths & States */
    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    public final Pose startPose = new Pose(44, 8, Math.toRadians(90));     // Red Far Launch Zone start
    public final Pose scorePose = new Pose(59, 81, Math.toRadians(133));    // Red goal scoring pose
    public final Pose parkPose = new Pose(45    , 40, Math.toRadians(270));

//    public final Pose startPose = new Pose(22, 122, Math.toRadians(135));     // Red Far Launch Zone start
//    public final Pose scorePose = new Pose(55, 80, Math.toRadians(131));    // Red goal scoring pose
//    public final Pose parkPose = new Pose(50, 130, Math.toRadians(270)); // Red Home (park)

    public Path scorePreload;
    public PathChain goPark;

    public enum pathingState { START, Firing, GO_PARK, READY }
    pathingState pathState = pathingState.READY;
    private boolean parkPathStarted = false;
    private boolean scorePathStarted = false;


    /**  Required OpMode Autonomous Control Methods  */

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        decBot.initRobot(hardwareMap);

        /**  Optional per-path tuning */
        shotsToFire = 4;
        MaxTimePark = 25.0;
        targetVelocity = 830;
        targetVelocityTwo = targetVelocity - 40;
        targetVelocityThree = targetVelocity + 100;
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();

        pathState = pathingState.START;
        currentState = FiringStates.START_DELAY;
        shotsFired = 0;
        startDelayStarted = false;
        fireDelayStarted = false;

        parkPathStarted = false;
        scorePathStarted = false;
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {

            case START:
                if (!scorePathStarted) {
                    follower.followPath(scorePreload);
                    scorePathStarted = true;
                }
                pathState = pathingState.Firing;
                break;

            case Firing:
                /**  If still driving to goal, optionally spin up early */
                if (follower.isBusy()) {
                    powerUpFlyWheels();
                    launchSequence();
                    break;
                }

                /**  Begin scoring session. Adjust for number of shots and time limit */
                if (currentState != FiringStates.IDLE) {
                    launchSequence();
                }

                /**  Edge Case Handling for Max Shots or Out of Autonomous Time  */
                boolean done = (currentState == FiringStates.IDLE);
                boolean timeToLeave = opmodeTimer.getElapsedTimeSeconds() >= MaxTimePark;

                if (done || timeToLeave) {
                    currentState = FiringStates.IDLE;
                    pathState = pathingState.GO_PARK;
                }
                break;

            case GO_PARK:
                /** Start the park path exactly once upon entering this state */
                if (!parkPathStarted) {
                    follower.followPath(goPark);
                    parkPathStarted = true;
                }
                powerUpFlyWheels();
                launchSequence();
                // When park path finishes, advance to READY
                if (parkPathStarted && !follower.isBusy()) {
                    pathState = pathingState.READY;
                }
                break;
//
            case READY:
                // Do nothing, keep robot safe
                launchSequence();
                break;
        }

        /** LED Driver for Gate Control */
        LEDDriver();

        /**  Telemetry: Include Base Telementry and add additional for Pathing */

        TelemetryOut();
        telemetry.addData("Pathing State", pathState);
        telemetry.addData("At goal?", !follower.isBusy());
        telemetry.addData("Auto Time (s)", "%.1f", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Leaving to park at (s)", MaxTimePark);
        telemetry.update();
    }

    @Override
    public void stop() { }

    /**  Pedro Pathing Control Methods  */

    public void buildPaths() {
        // Start Pose -> Score Pose
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Score Pose -> Park Home Pose
        goPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }
}
