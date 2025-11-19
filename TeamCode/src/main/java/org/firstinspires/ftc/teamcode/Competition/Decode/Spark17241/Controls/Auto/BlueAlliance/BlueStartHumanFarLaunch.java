package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.BlueAlliance;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;


@Autonomous(name = "Blue:Start Human Far Launch", group = "Drive")
public class BlueStartHumanFarLaunch extends AutoMain {
//
    /**  Pedro Pathing Variables, Poses, Paths & States */
    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    /*public final Pose startPose = new Pose(44, 10, Math.toRadians(90));     // start pos
    public final Pose scoreFarPose = new Pose(60, 20, Math.toRadians(111));    // blue shoot far
    public final Pose parkPose = new Pose(56, 35, Math.toRadians(0));*/ // Red Home (park)

    public final Pose startPose = new Pose(44, 10, Math.toRadians(90));     // start pos
    public final Pose scoreFarPose = new Pose(60, 20, Math.toRadians(112));    // blue shoot far
    public final Pose parkPose = new Pose(56, 35, Math.toRadians(0));

    public Path scorePreload;
    public PathChain goPark;

    public enum pathingState { START, Shooting, GO_PARK, READY }
    pathingState pathState = pathingState.READY;
    private boolean parkPathStarted = false;


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
        maxShots = 4;                       // Adjust for shot attempts
        parkLeaveTime = 25.0;               // Adjust if this path is long
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();

        pathState = pathingState.START;
        scoringState = scoreState.IDLE;
        launchZone = LaunchZone.NONE;

        autoScoreComplete = false;
        shotsFired = 0;
        parkPathStarted = false;
    }

    @Override
    public void loop() {
        follower.update();
        LEDDriver();
        switch (pathState) {

            case START:
                follower.followPath(scorePreload);
                pathState = pathingState.Shooting;
                break;

            case Shooting:
                /**  If still driving to goal, optionally spin up early */
                if (follower.isBusy()) {
                    launchZone = LaunchZone.FAR;
                    onLoopStart();
                    updateFlywheelAndGate();
                    break;
                }

                /**  Begin scoring session. Adjust for number of shots and time limit */
                if (!isScoringActive()) {

                    startScoring(LaunchZone.FAR, 4, 9.0, opmodeTimer.getElapsedTimeSeconds());
                }

                /**  Edge Case Handling for Max Shots or Out of Autonomous Time  */
                boolean done = updateScoring(opmodeTimer.getElapsedTimeSeconds());
                boolean timeToLeave = opmodeTimer.getElapsedTimeSeconds() >= parkLeaveTime;

                if (done || timeToLeave) {
                    stopScoring(); // safe even if already inactive
                    pathState = pathingState.GO_PARK;
                }
                break;

            case GO_PARK:
                /** Start the park path exactly once upon entering this state */
                if (!parkPathStarted) {
                    follower.followPath(goPark);
                    parkPathStarted = true;
                }
                launchZone = LaunchZone.NONE;    // spin down while driving to park
                onLoopStart();
                updateFlywheelAndGate();         // harmless when NONE
                // When park path finishes, advance to READY
                if (parkPathStarted && !follower.isBusy()) {
                    pathState = pathingState.READY;
                }
                break;
//
            case READY:
                // Do nothing, keep robot safe
                onLoopStart();
                updateFlywheelAndGate();
                break;
        }
        /** LED Driver for Gate Control */
        LEDDriver();

        /**  Telemetry: Include Base Telementry and add additional for Pathing */

        baseTelemetry();
        telemetry.addData("Pathing State", pathState);
        telemetry.addData("At goal?", !follower.isBusy());
        telemetry.addData("Auto Time (s)", "%.1f", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Leaving to park at (s)", parkLeaveTime);
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

