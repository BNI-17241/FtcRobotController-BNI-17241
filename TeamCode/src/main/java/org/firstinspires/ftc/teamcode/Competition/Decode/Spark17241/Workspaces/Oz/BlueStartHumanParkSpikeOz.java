package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;

@Autonomous(name = "Blue:Start Human:Park  OZ", group = "Drive")
// Extend the new AutoMain_oz class
public class BlueStartHumanParkSpikeOz extends AutoMain_oz {
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

        shotsToFire = 4;
        MaxTimePark = 25.0;
        targetVelocity = 890;
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

                if (follower.isBusy()) {
                    launchSequence();
                    break;
                }

                boolean done = (currentState == FiringStates.IDLE);
                boolean timeToLeave = opmodeTimer.getElapsedTimeSeconds() >= MaxTimePark;

                if (done || timeToLeave) {
                    currentState = FiringStates.IDLE;
                    pathState = pathingState.GO_PARK;
                }
                launchSequence();
                break;

            case GO_PARK:
                if (!parkPathStarted) {
                    follower.followPath(goPark);
                    parkPathStarted = true;
                }
                launchSequence();
                if (parkPathStarted && !follower.isBusy()) {
                    pathState = pathingState.READY;
                }
                break;
            case READY:
                launchSequence();
                break;
        }


        TelemetryOut();
        telemetry.addData("Pathing State", pathState);
        telemetry.addData("At goal?", !follower.isBusy());
        telemetry.addData("Auto Time (s)", "%.1f", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Leaving to park at (s)", MaxTimePark);
        telemetry.update();
    }

    @Override
    public void stop() { }



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
