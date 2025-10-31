package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;

@Autonomous(name = "Red:Far Launch:Park Home", group = "Drive")
public class autoFarLaunchRedAllianceParkHome extends AutoMain {

    // ********** Pedro Pathing Variables, Poses, Paths & States *******************
    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    public final Pose startPose = new Pose(56, 8, Math.toRadians(90));     // Red Far Launch Zone start
    public final Pose scorePose = new Pose(90, 90, Math.toRadians(50));    // Red goal scoring pose
    public final Pose parkHomePose = new Pose(50, 40, Math.toRadians(180)); // Red Home (park)

    public Path scorePreload;
    public PathChain goPark;

    public enum pathingState { START, SCORE_PRELOAD, GO_PARK, READY }
    pathingState pathState = pathingState.READY;

    //****************  Required OpMode Auto Control Methods  ********************

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        decBot.initRobot(hardwareMap);

        // Optional per-path tuning:
        maxShots = 4;
        parkLeaveTime = 25.0;  // adjust if this path is long
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
                // If still driving to goal, optionally spin up early
                if (follower.isBusy()) {
                    launchZone = LaunchZone.NEAR;
                    onLoopStart();
                    updateFlywheelAndGate();
                    break;
                }

                // Arrived: begin scoring session once
                if (!isScoringActive()) {
                    // NEAR zone, 4 shots, up to ~8 seconds at goal (tune if needed)
                    startScoring(LaunchZone.NEAR, 4, 8.0, opmodeTimer.getElapsedTimeSeconds());
                }

                // Continue scoring until finished or time to leave to park
                boolean done = updateScoring(opmodeTimer.getElapsedTimeSeconds());
                boolean timeToLeave = opmodeTimer.getElapsedTimeSeconds() >= parkLeaveTime;

                if (done || timeToLeave) {
                    stopScoring(); // safe even if already inactive
                    pathState = pathingState.GO_PARK;
                }
                break;

            case GO_PARK:
                // Start the park path once (only when we first enter)
                if (follower.isBusy() == false) {
                    follower.followPath(goPark);
                }
                launchZone = LaunchZone.NONE;    // spin down while driving to park
                onLoopStart();
                updateFlywheelAndGate();         // harmless when NONE
                if (!follower.isBusy()) {
                    pathState = pathingState.READY;
                }
                break;

            case READY:
                // Do nothing, keep robot safe
                onLoopStart();
                updateFlywheelAndGate();
                break;
        }

        // Telemetry
        baseTelemetry();
        telemetry.addData("Pathing State", pathState);
        telemetry.addData("At goal?", !follower.isBusy());
        telemetry.addData("Auto Time (s)", "%.1f", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Leaving to park at (s)", parkLeaveTime);
        telemetry.update();
    }

    @Override
    public void stop() { }

    //****************  Pedro Pathing Control Methods  ********************

    public void buildPaths() {
        // Start -> Score
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Score -> Park Home
        goPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkHomePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkHomePose.getHeading())
                .build();
    }
}

