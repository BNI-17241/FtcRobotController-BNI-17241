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

    public enum pathingState { START, SCORE_PRELOAD, PARK_HOME, READY }
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

    int shotCounter = 1;
    int shotCapacity = 4;
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
        setPathState(pathingState.SCORE_PRELOAD);
        scoringState = scoreState.READY;
        boolean autoScoreComplete = false;
    }


    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        flyWheelControl(LaunchZone.NEAR);
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
                scoringState = scoreState.IDLE;
                setPathState(pathingState.SCORE_PRELOAD);
                break;

            case SCORE_PRELOAD:
                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                if (!follower.isBusy() || autoScoreComplete || pathTimer.getElapsedTimeSeconds() > 20) {
                    flyWheelControl(LaunchZone.NONE);
                    scoringState = scoreState.EMPTY;
                    follower.followPath(goPark, true);
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
                if (!autoScoreComplete) {
                    scoringState = scoreState.WAIT_FOR_GATE;
                }
                break;

            case WAIT_FOR_GATE:
                decBot.feederWheel.setPower(0);
                if (inGate && shotCounter < shotCapacity) {
                    // Apply brief boost and feed
                    targetVelocity = nominalTarget * boostFactor;
                    timer.reset();
                    decBot.feederWheel.setPower(feederPower);
                    scoringState = scoreState.FEEDING;
                }
                else {
                    scoringState = scoreState.EMPTY;
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
                    shotCounter += 1;
                    scoringState = scoreState.RECOVERING;
                }
                break;

            case RECOVERING:
                // Give the wheel a short window to re-settle; you can also re-arm immediately.
                if (timer.milliseconds() >= boostMs) {
                    scoringState = scoreState.IDLE;
                }
                break;

            case READY:
                autoScoreComplete = false;
                break;

            case EMPTY:
                autoScoreComplete = true;
                break;
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
        telemetry.update();
    }

}
