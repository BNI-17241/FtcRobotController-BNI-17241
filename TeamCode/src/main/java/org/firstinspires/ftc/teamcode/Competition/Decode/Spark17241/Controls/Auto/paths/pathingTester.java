package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;

@Autonomous(name = "Pathing Tester", group = "Lab")
public class pathingTester extends OpMode {

    // ********** Pedro Pathing Variables, Poses, Paths & States *******************

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Used Visualizer to determine pose
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of robot (Red Far Launch Zone).
    private final Pose scorePose = new Pose(90, 90, Math.toRadians(50)); // Red Goal Scoring Pose of robot.
    private final Pose parkHomePose = new Pose(50, 40, Math.toRadians(180)); // Red Home

    private Path scorePreload;
    private PathChain goPark, scorePickup1;

    public enum pathingState { START, SCORE_PRELOAD, SCORE_COUNT, GO_PARK, READY }
    pathingState pathState = pathingState.READY;

    // ********** FlyWheel and Intake Control Variables, Constants and States *******************

    // Flywheel & Feed Wheel Variables
    public double targetVelocity = 0;

    // Velocity gate
    double gatePercent = 0.05;            // ±5`     % gate window

    // Feed action
    double feederPower = 1.0;             // power for feeder wheel (0..1)
    long   feedMs = 700;                  // how long to run feeder

    // Shot-drop compensation (temporary target bump while feeding)
    double boostFactor = 1.02;            // +2% target during feed
    long   boostMs = 180;                 // usually ~150–250 ms

    // Feeder FlyWheel Gate State Control
    enum scoreState { READY, IDLE, WAIT_FOR_GATE, FEEDING, RECOVERING, EMPTY }
    scoreState scoringState = scoreState.IDLE;
    ElapsedTime timer = new ElapsedTime();

    int maxShots = 4;
    int shotsFired = 0;                /
    scoreState prevScoringState = scoreState.IDLE;
    boolean parkPathStarted = false;
    double parkLeaveTime = 25.0;

    boolean autoScoreComplete;
    enum LaunchZone {FAR, NEAR, NONE}
    LaunchZone launchZone = LaunchZone.NEAR;


    double nominalTarget = 0;             // remembers non-boosted target
    double tolerance; // floor to 10 ticks per secibd
    boolean leftInGateStatus = false;
    boolean rightInGateStatus = false;
    boolean inGate = false;

    double currentVelocityLeft;
    double currentVelocityRight;

    // Constructor for Physical Robot
    public DecodeBot decBot = new DecodeBot();

    //****************  Required OpMode Auto Control Methods  ********************

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }
    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathState = pathingState.START;
        pathTimer.resetTimer();
        scoringState = scoreState.READY;
        launchZone = LaunchZone.NONE;
        autoScoreComplete = false;
        shotsFired = 0;
        parkPathStarted = false;
    }


    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        flyWheelControl(launchZone);
        prevScoringState = scoringState;
        automaticFeedControl();
        telemetryUpdate();

    }

    @Override
    public void stop() {}


    //****************  Pedro Pathing Control Methods  ********************

    public void buildPaths() {
        /* From Start and To Score Preload Path. We are using a BezierLine. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* From Score and Tp Park Home  Path.  We are using a single path with a BezierLine. */
        goPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkHomePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkHomePose.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case START:
                follower.followPath(scorePreload);
                launchZone = LaunchZone.NEAR;               // Starts the spin up on the way
                scoringState = scoreState.IDLE;             // not arming until we arrive
                pathState = pathingState.SCORE_PRELOAD;
                pathTimer.resetTimer();
                break;

            case SCORE_PRELOAD:
                // if we are still driving, we keep spinning up flywheels
                if (!follower.isBusy() ) {
                    launchZone = LaunchZone.NEAR;
                    break;
                }

                // We've arrived at the scoring pose: run inner scoring until done/time
                launchZone = LaunchZone.NEAR;          // keep flywheels hot while scoring

                // Arm the inner state machine if it's idle/ready
                if (scoringState == scoreState.IDLE || scoringState == scoreState.READY) {
                    scoringState = scoreState.WAIT_FOR_GATE;
                }

                // Exit condition: out of time to still park OR finished 4 shots
                boolean timeToLeave = opmodeTimer.getElapsedTimeSeconds() >= parkLeaveTime;
                if (shotsFired >= maxShots || timeToLeave) {
                    launchZone = LaunchZone.NONE;      // spin down
                    scoringState = scoreState.EMPTY;   // freeze feeder
                    pathState = pathingState.GO_PARK;
                    parkPathStarted = false;           // allow starting the park path
                    pathTimer.resetTimer();
                }
                break;

            case GO_PARK:
                // Kick off the park path once
                if (!parkPathStarted) {
                    follower.followPath(goPark);
                    parkPathStarted = true;
                }
                launchZone = LaunchZone.NONE;          // flywheels off while driving to park

                if (!follower.isBusy()) {
                    pathState = pathingState.READY;
                    pathTimer.resetTimer();
                }
                break;

            case READY:
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(pathingState pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    //****************  Scoring Control Methods  ********************

    // Fly Wheel Control
    public void flyWheelControl(LaunchZone zone) {

        if (zone == LaunchZone.NEAR) { targetVelocity = 1003; }
        if (zone == LaunchZone.FAR) { targetVelocity = 1125; }
        if (zone == LaunchZone.NONE) { targetVelocity = 0; }

        decBot.flylaunch(targetVelocity);

        // Keep nominalTarget synced unless we’re in a boost
        if (scoringState == scoreState.IDLE || scoringState == scoreState.WAIT_FOR_GATE) {
            nominalTarget = targetVelocity;
        }

        // Always command velocity each loop
        decBot.leftFlyWheel.setVelocity(targetVelocity);
        decBot.rightFlyWheel.setVelocity(targetVelocity);

        // ===== Read velocities & gate =====
        currentVelocityLeft = decBot.leftFlyWheel.getVelocity();
        currentVelocityRight = decBot.rightFlyWheel.getVelocity();

        tolerance = Math.max(10.0, Math.abs(nominalTarget) * gatePercent); // floor to 10 ticks per secibd
        leftInGateStatus  = Math.abs(currentVelocityLeft - nominalTarget) <= tolerance;
        rightInGateStatus = Math.abs(currentVelocityRight - nominalTarget) <= tolerance;
        inGate = leftInGateStatus && rightInGateStatus;


    }

    public void automaticFeedControl() {
        // ===== State machine =====
        switch (scoringState) {
            case IDLE:
                decBot.feederWheel.setPower(0);
                break;

            case WAIT_FOR_GATE:
                decBot.feederWheel.setPower(0);
                if (inGate ) {
                    nominalTarget = targetVelocity;         // lock in current nominal
                    targetVelocity = nominalTarget * boostFactor;
                    timer.reset();
                    decBot.feederWheel.setPower(feederPower);
                    scoringState = scoreState.FEEDING;
                }
                break;

            case FEEDING:
                // Maintain boost while feeding
                targetVelocity = nominalTarget * boostFactor;
                if (timer.milliseconds() >= feedMs) {
                    decBot.feederWheel.setPower(0);
                    // Start recovery (let wheel return to nominal target)
                    targetVelocity = nominalTarget;
                    timer.reset();
                    scoringState = scoreState.RECOVERING;
                }
                break;

            case RECOVERING:
                // Give the wheel a short window to re-settle; you can also re-arm immediately.
                if (timer.milliseconds() >= boostMs) {
                    scoringState = scoreState.WAIT_FOR_GATE;
                }
                break;

            case EMPTY:
                autoScoreComplete = true;
                break;
        }
        // ===== Count shot when FEEDING completes (first frame entering RECOVERING) =====
        if (prevScoringState == scoreState.FEEDING && scoringState == scoreState.RECOVERING) {
            shotsFired++;
        }

    }

    // Feedback to Driver Hub for debugging
    public void telemetryUpdate() {
        telemetry.addData("Pathing State", pathState);
        telemetry.addData("Launching State", scoringState);
        telemetry.addData("inGate Status", "%b | %b", leftInGateStatus, rightInGateStatus);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addData("Target (nominal velocity)", nominalTarget);
        telemetry.addData("Gate Tolerance ±%", gatePercent * 100.0);
        telemetry.addData("Left Fly Wheel velocity", currentVelocityLeft);
        telemetry.addData("Right Fly Wheel velocity", currentVelocityRight);
        telemetry.addData("Shots Fired / Max", "%d / %d", shotsFired, maxShots);
        telemetry.addData("Prev->Curr ScoreState", "%s -> %s", prevScoringState, scoringState);
        telemetry.addData("At goal?", !follower.isBusy());
        telemetry.addData("Auto Time (s)", "%.1f", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Leaving to park at (s)", parkLeaveTime);
        telemetry.update();
    }

}
