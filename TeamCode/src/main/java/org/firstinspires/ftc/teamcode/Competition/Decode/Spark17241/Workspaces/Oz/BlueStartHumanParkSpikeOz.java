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

//@Disabled
@Autonomous(name = "Blue:Start Human:Park Spike Oz", group = "Drive")
public class BlueStartHumanParkSpikeOz extends AutoMain_oz {

    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    public final Pose startPose = new Pose(44, 8, Math.toRadians(90));
    public final Pose scorePose = new Pose(59, 81, Math.toRadians(133));
    public final Pose parkPose = new Pose(45, 40, Math.toRadians(0));

    public Path scorePreload;
    public PathChain goPark;

    public enum pathingState { START, SCORE_PRELOAD, GO_PARK, READY }
    pathingState pathState = pathingState.READY;
    private boolean parkPathStarted = false;


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        decBot.initRobot(hardwareMap);

        shotsToFire = 3;
        MaxTimePark = 25.0;
        targetVelocity = 950;
        targetVelocityTwo = targetVelocity - 75;
        targetVelocityThree = targetVelocity - 100;
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
                    powerUpFlyWheels();
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
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        goPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }
}
