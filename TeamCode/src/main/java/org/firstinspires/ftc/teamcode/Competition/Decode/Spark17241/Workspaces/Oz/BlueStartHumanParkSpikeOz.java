package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;

@Autonomous(name = "OZ Test BSHP", group = "Drive")
public class BlueStartHumanParkSpikeOz extends AutoMainOzV2{
//
    /**  Pedro Pathing Variables, Poses, Paths & States */
    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    public final Pose startPose = new Pose(44, 8, Math.toRadians(90));     // Red Far Launch Zone start
    public final Pose scorePose = new Pose(59, 81, Math.toRadians(133));    // Red goal scoring pose // 80 x 80
    public final Pose parkPose = new Pose(45, 40, Math.toRadians(0)); // Red Home (park)

    public Path scorePreload;
    public PathChain goPark;

    public enum pathingState { START, SCORE_PRELOAD, GO_PARK, READY }
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
        maxShots = 6 ;                       // Adjust for shot attempts
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

        firstShotVelocity = 1000;
        secountShotVelocity = 500;
        thirdShotVelocity = 100;
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {

            case START:
                follower.followPath(scorePreload);
                pathState = pathingState.SCORE_PRELOAD;
                break;

            case SCORE_PRELOAD:
                /**  If still driving to goal, optionally spin up early */
                if (follower.isBusy()) {
                    launchZone = LaunchZone.NEAR;
                    onLoopStart();
                    updateFlywheelAndGate(firstShotVelocity, secountShotVelocity, thirdShotVelocity);
                    break;
                }

                /**  Begin scoring session. Adjust for number of shots and time limit */
                if (!isScoringActive()) {

                    startScoring(LaunchZone.NEAR, 4, 10.0, opmodeTimer.getElapsedTimeSeconds());
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
                updateFlywheelAndGate(firstShotVelocity, secountShotVelocity, thirdShotVelocity);         // harmless when NONE
                // When park path finishes, advance to READY
                if (parkPathStarted && !follower.isBusy()) {
                    pathState = pathingState.READY;
                }
                break;
//
            case READY:
                // Do nothing, keep robot safe
                onLoopStart();
                updateFlywheelAndGate(firstShotVelocity, secountShotVelocity, thirdShotVelocity);
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
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Score Pose -> Park Home Pose
        goPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }
}

