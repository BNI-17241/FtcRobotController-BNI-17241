package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.BlueAlliance.BlueStartHumanParkSpike;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;

//@Disabled
@Autonomous(name = "tester Oz Blue:Start Human:Park Spike", group = "Drive")
public class BlueStartHumanParkSpike_oz extends AutoMain {
//
    /**  Pedro Pathing Variables, Poses, Paths & States */
    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    //poses

    public final Pose startPose = new Pose(44, 8, Math.toRadians(90));     // Red Far Launch Zone start
    public final Pose scorePose = new Pose(59, 81, Math.toRadians(135));    // Red goal scoring pose // 80 x 80
    public final Pose LineUpPose = new Pose(40, 35, Math.toRadians(180)); // Lines up with balls
    public final Pose collectPose = new Pose(20, 35, Math.toRadians(210)); // collects balls
    public final Pose pushBallPose = new Pose(10, 20, Math.toRadians(270)); // push ball into human area
    public final Pose backUpPose = new Pose(10, 40, Math.toRadians(270)); // backs up after pushing balls in

//
    public Path scorePreload;
    public PathChain LineUp;
    public PathChain collect;
    public PathChain pushBall;
    public PathChain backUp;

    public enum pathingState { START, SCORE_PRELOAD, LINE_UP, COLLECT, PUSH_BALL, BACK_UP, READY }
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
                    updateFlywheelAndGate();
                    break;
                }

                /**  Begin scoring session. Adjust for number of shots and time limit */
                if (!isScoringActive()) {
                    startScoring(LaunchZone.NEAR, 4, 8.0, opmodeTimer.getElapsedTimeSeconds());
                }

                /**  Edge Case Handling for Max Shots or Out of Autonomous Time  */
                boolean done = updateScoring(opmodeTimer.getElapsedTimeSeconds());
                boolean timeToLeave = opmodeTimer.getElapsedTimeSeconds() >= parkLeaveTime;

                if (done || timeToLeave) {
                    stopScoring(); // safe even if already inactive
                    pathState = pathingState.LINE_UP;
                }
                break;

            case LINE_UP:
                launchZone = LaunchZone.NONE;    // spin down while driving to park
                onLoopStart();
                updateFlywheelAndGate();         // harmless when NONE
                // When park path finishes, advance to READY
                if (!follower.isBusy()) {
                    pathState = pathingState.COLLECT;
                }
                break;
            case COLLECT:
                if(!follower.isBusy()) {
                    follower.followPath(collect,true);
                    pathState = pathingState.PUSH_BALL;
                }
            case PUSH_BALL:
                if(!follower.isBusy()) {
                    follower.followPath(pushBall,true);
                    pathState = pathingState.BACK_UP;
                }
            case BACK_UP:
                /** Start the park path exactly once upon entering this state */
                if (!parkPathStarted) {
                    follower.followPath(LineUp);
                    parkPathStarted = true;
                }
                if(!follower.isBusy()) {
                    follower.followPath(backUp, true);
                    pathState = pathingState.READY;
                }
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
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Score Pose -> Line up pose
        LineUp = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, LineUpPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), LineUpPose.getHeading())
                .build();
        // Line up pose -> Collect pose
        collect = follower.pathBuilder()
                .addPath(new BezierLine(LineUpPose,collectPose))
                .setLinearHeadingInterpolation(LineUpPose.getHeading(), collectPose.getHeading())
                .build();
        // Collect pose -> pushball pose
        pushBall = follower
                .pathBuilder()
                .addPath(new BezierLine(collectPose, pushBallPose))
                .setLinearHeadingInterpolation(collectPose.getHeading(), pushBallPose.getHeading())
                .build();
        // Push ball pose -> back up (final pose)
        backUp = follower
                .pathBuilder()
                .addPath(new BezierLine(pushBallPose, backUpPose))
                .setLinearHeadingInterpolation(pushBallPose.getHeading(), backUpPose.getHeading())
                .build();


    }
}

