package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.Old.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.AutoMain_NewAndrew;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Red:Start Human Far Launch Meet Two", group = "Drive")
public class RedStartHumanFarLaunchDecodeMeetTwo extends AutoMain_NewAndrew {
//
    /**  Pedro Pathing Variables, Poses, Paths & States */
    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    public final Pose startPose = new Pose(100, 10, Math.toRadians(90));     // start pos
    public final Pose scoreFarPose = new Pose(84, 20, Math.toRadians(62));    // blue shoot far
    public final Pose parkPose = new Pose(90, 35, Math.toRadians(180)); // Red Home (park)

    public Path scorePreload;
    public PathChain goPark;

    public enum pathingState { START, Shooting, GO_PARK, READY }
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
        targetVelocity = 1090;
        targetVelocityTwo = targetVelocity - 75;
        targetVelocityThree = targetVelocity - 100;
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        max_time_timer.resetTimer();

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
        LEDDriver();
        switch (pathState) {

            case START:
                if (!scorePathStarted) {
                    follower.followPath(scorePreload);
                    scorePathStarted = true;
                }
                pathState = pathingState.Shooting;
                break;

            case Shooting:
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
        scorePreload = new Path(new BezierLine(startPose, scoreFarPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scoreFarPose.getHeading());

        // Score Pose -> Park Home Pose
        goPark = follower.pathBuilder()
                .addPath(new BezierLine(scoreFarPose, parkPose))
                .setLinearHeadingInterpolation(scoreFarPose.getHeading(), parkPose.getHeading())
                .build();
    }
}
