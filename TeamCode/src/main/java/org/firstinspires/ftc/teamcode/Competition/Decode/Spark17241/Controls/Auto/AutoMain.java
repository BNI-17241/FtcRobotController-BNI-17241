package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;

/**
 * Abstract base OpMode for Autonomous pathing.
 * Holds ALL shooter/flywheel/feeder logic so per-path OpModes only define paths and call the scoring session.
 *
 */
public abstract class AutoMain extends OpMode {

    // ===== Scoring Constants (independent of pathing states) =====
    public enum scoreState { READY, IDLE, WAIT_FOR_GATE, FEEDING, RECOVERING, EMPTY }
    public enum LaunchZone { FAR, NEAR, NONE }

    // ===== Flywheel / Feeder Variables =====
    protected double targetVelocity = 0;
    protected double gatePercent = 0.05;     // ±5% gate window
    protected double feederPower = 1.0;      // feeder wheel power (0..1)
    protected long   feedMs = 700;           // ms to run feeder during a shot
    protected double boostFactor = 1.02;     // temporary +2% velocity during feed
    protected long   boostMs = 180;          // recovery delay after feed

    // ===== Scoring state machine =====
    protected scoreState scoringState = scoreState.IDLE;
    protected scoreState prevScoringState = scoreState.IDLE;
    protected ElapsedTime timer = new ElapsedTime();

    // ===== Global counters / flags =====
    protected int maxShots = 4;
    protected int shotsFired = 0;
    protected boolean autoScoreComplete;

    // ===== Launching Variables  =====
    protected LaunchZone launchZone = LaunchZone.NEAR;
    protected double nominalTarget = 0;
    protected double tolerance;
    protected boolean leftInGateStatus = false;
    protected boolean rightInGateStatus = false;
    protected boolean inGate = false;
    protected double currentVelocityLeft;
    protected double currentVelocityRight;

    // ===== Parking timing helper (subclasses can override) =====
    protected double parkLeaveTime = 25.0;  // seconds into auto to guarantee time to park

    // ===== Robot instance (motors/sensors accessed here) =====
    protected DecodeBot decBot = new DecodeBot();

    // --------------------------------------------------------------------------------------------------
    // Reusable base methods
    // --------------------------------------------------------------------------------------------------

    // Called at the very start of your OpMode.loop() in subclasses to snapshot the previous scoring state.
    protected void onLoopStart() {
        prevScoringState = scoringState;
    }
    // Flywheel velocity control and gate computation. Subclasses set `launchZone` before calling.
    protected void updateFlywheelAndGate() {
        if (launchZone == LaunchZone.NEAR) {
            targetVelocity = 1003;
        } else if (launchZone == LaunchZone.FAR) {
            targetVelocity = 1125;
        } else {
            targetVelocity = 0;
        }

        decBot.flylaunch(targetVelocity);

        if (scoringState == scoreState.IDLE || scoringState == scoreState.WAIT_FOR_GATE) {
            nominalTarget = targetVelocity;
        }

        decBot.leftFlyWheel.setVelocity(targetVelocity);
        decBot.rightFlyWheel.setVelocity(targetVelocity);

        currentVelocityLeft = decBot.leftFlyWheel.getVelocity();
        currentVelocityRight = decBot.rightFlyWheel.getVelocity();

        tolerance = Math.max(10.0, Math.abs(nominalTarget) * gatePercent);
        leftInGateStatus  = Math.abs(currentVelocityLeft - nominalTarget) <= tolerance;
        rightInGateStatus = Math.abs(currentVelocityRight - nominalTarget) <= tolerance;
        inGate = leftInGateStatus && rightInGateStatus;
    }

    // Feeder Wheel State Machine. Does not count shots; use `justCompletedFeed()` for edge-detect.
    protected void runAutoFeederCycle() {
        switch (scoringState) {
            case IDLE:
                decBot.feederWheel.setPower(0);
                break;

            case WAIT_FOR_GATE:
                decBot.feederWheel.setPower(0);
                if (inGate) {
                    nominalTarget = targetVelocity;
                    targetVelocity = nominalTarget * boostFactor;
                    timer.reset();
                    decBot.feederWheel.setPower(feederPower);
                    scoringState = scoreState.FEEDING;
                }
                break;

            case FEEDING:
                targetVelocity = nominalTarget * boostFactor;
                if (timer.milliseconds() >= feedMs) {
                    decBot.feederWheel.setPower(0);
                    targetVelocity = nominalTarget;
                    timer.reset();
                    scoringState = scoreState.RECOVERING;
                }
                break;

            case RECOVERING:
                if (timer.milliseconds() >= boostMs) {
                    scoringState = scoreState.WAIT_FOR_GATE;
                }
                break;

            case READY:
                decBot.feederWheel.setPower(0);
                break;

            case EMPTY:
                decBot.feederWheel.setPower(0);
                break;
        }
    }

    // True exactly when FEEDING → RECOVERING (one shot finished). */
    protected boolean justCompletedFeed() {
        return (prevScoringState == scoreState.FEEDING) && (scoringState == scoreState.RECOVERING);
    }

    protected boolean isScoringActive() {
        return scoring.active;
    }

    /** Optional shared telemetry */
    protected void baseTelemetry() {
        telemetry.addData("LaunchZone", launchZone);
        telemetry.addData("Score State", scoringState);
        telemetry.addData("Prev->Curr", "%s -> %s", prevScoringState, scoringState);
        telemetry.addData("Shots", "%d / %d", shotsFired, maxShots);
        telemetry.addData("Gate", "%b | %b", leftInGateStatus, rightInGateStatus);
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Nominal Target", nominalTarget);
        telemetry.addData("Gate Tol ±%", gatePercent * 100.0);
        telemetry.addData("L Vel", currentVelocityLeft);
        telemetry.addData("R Vel", currentVelocityRight);
    }

    // --------------------------------------------------------------------------------------------------
    // Scoring Session Orchestrator (template-method style)
    // --------------------------------------------------------------------------------------------------

    protected static class ScoringSession {
        boolean active = false;
        int targetShots = 0;
        int shotsFiredAtStart = 0;
        double timeLimitSec = 0;
        double startedAtSec = 0;
        LaunchZone zone = LaunchZone.NONE;
    }

    protected ScoringSession scoring = new ScoringSession();

    // Call once when you arrive at the shooting pose. */
    protected void startScoring(LaunchZone zone, int shots, double timeLimitSec, double nowSec) {
        scoring.active = true;
        scoring.zone = zone;
        scoring.targetShots = shots;
        scoring.shotsFiredAtStart = shotsFired;
        scoring.timeLimitSec = timeLimitSec;
        scoring.startedAtSec = nowSec;

        launchZone = zone;
        scoringState = scoreState.WAIT_FOR_GATE;
    }

    /**
     * Call every loop while scoring.active. Returns true when done (hit count or timed out).
     * Encapsulates flywheel control + feeder FSM + shot edge counting.
     */
    protected boolean updateScoring(double nowSec) {
        if (!scoring.active) return true;

        onLoopStart();
        updateFlywheelAndGate();
        runAutoFeederCycle();

        if (justCompletedFeed()) {
            shotsFired++;
        }

        boolean hitCount = (shotsFired - scoring.shotsFiredAtStart) >= scoring.targetShots;
        boolean timedOut = (nowSec - scoring.startedAtSec) >= scoring.timeLimitSec;

        if (hitCount || timedOut) {
            launchZone = LaunchZone.NONE;
            scoringState = scoreState.EMPTY;
            scoring.active = false;
            autoScoreComplete = true;
            return true;
        }
        return false;
    }

    /** Optional: force stop (if pathing needs to bail early). */
    protected void stopScoring() {
        scoring.active = false;
        launchZone = LaunchZone.NONE;
        scoringState = scoreState.EMPTY;
    }
}