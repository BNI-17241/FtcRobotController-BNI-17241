package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;

/**
 * Abstract base OpMode for Autonomous pathing.
 * Holds ALL shooter/flywheel/feeder logic so per-path OpModes only define paths and call the scoring methods.
 *
 */

public abstract class AutoMain_oz extends OpMode {

    /** LimeLight and April Tags Variables, Constants  */


    /**  Flywheel / Feeder Variables  */
    protected double targetVelocity = 0;
    protected double gatePercent = 0.02;     // ±5% gate window
    protected double feederPower = 1.0;      // feeder wheel power (0..1)
    protected long   feedMs = 500;           // ms to run feeder during a shot
    protected double boostFactor = 1.0;     // temporary +2% velocity during feed
    protected long   boostMs = 180;          // recovery delay after feed

    double minTimeBetweenShots = 2.0;
    double currentCountDownShot = 0.0;

    boolean startCount = false;
    public ElapsedTime mRuntime = new ElapsedTime();



    /**  Scoring State Machine Variables and Constants (Independent of Pathing States) ===== */
    protected enum scoreState { READY, IDLE, WAIT_FOR_GATE, WAITINGONTIME, FEEDING, RECOVERING, EMPTY }
    protected scoreState scoringState = scoreState.IDLE;
    protected scoreState prevScoringState = scoreState.IDLE;
    protected ElapsedTime timer = new ElapsedTime();

    /**  Global counters / Flags to tracker number of shots on goal */
    protected int maxShots = 5;
    protected int shotsFired = 0;
    protected boolean autoScoreComplete;

    /**  Launching Fly Wheel and Feeder Wheel Variables  */
    protected enum LaunchZone { FAR, NEAR, NONE }
    protected LaunchZone launchZone = LaunchZone.NEAR;

    protected double nominalTarget = 0;
    protected double tolerance;
    protected boolean leftInGateStatus = false;
    protected boolean rightInGateStatus = false;
    protected boolean inGate = false;
    protected double currentVelocityLeft;
    protected double currentVelocityRight;

    /** Parking timing helper  */
    protected double LastTimeToLeave = 25.0;  // seconds into auto to guarantee time to park
    protected double delayTime = 5.0;

    /**  Robot instance (motors/sensors accessed here) */
    protected DecodeBot decBot = new DecodeBot();

    /** Called at the start of OpMode.loop() to snapshot the previous scoring state. */
    protected void onLoopStart() {
        prevScoringState = scoringState;
    }


    /** Flywheel Velocity Control and Gate Control based on Launch Zone */
    protected void updateFlywheelAndGate() {
        if (launchZone == LaunchZone.NEAR) {
            targetVelocity = 890;                        //based on feedback 11/1
        } else if (launchZone == LaunchZone.FAR) {
            targetVelocity = 1093;                         //based on feedback 11/1
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

    /** Feeder Wheel State Machine. Uses `justCompletedFeed()` for edge-detect.    OZ says(I know this was written with chat)*/
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
                        scoringState = scoreState.WAITINGONTIME;



                }
                break;
            case WAITINGONTIME:
                if (currentCountDownShot <= 0){
                    currentCountDownShot = minTimeBetweenShots;
                    scoringState = scoreState.FEEDING;
                }
                else {
                    if (!startCount) {
                        startCount = true;
                        mRuntime.reset();
                    }
                    if (mRuntime.time() >= minTimeBetweenShots) {
                        currentCountDownShot = 0;
                        startCount = false;
                    }
                }
            case FEEDING:
                decBot.feederWheel.setPower(feederPower);
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
                    scoringState = scoreState.IDLE;
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

    /**  True when  FEEDING → RECOVERING (one shot finished). */
    protected boolean justCompletedFeed() {
        return (prevScoringState == scoreState.FEEDING) && (scoringState == scoreState.RECOVERING);
    }

    /**  True when Scoring process is active. */
    protected boolean isScoringActive() {
        return scoring.active;
    }

    /** Shared telemetry */
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

    /** Scoring Session Tracking / Orchestrator */

    protected static class ScoringSession {
        boolean active = false;
        int targetShots = 0;
        int shotsFiredAtStart = 0;
        double timeLimitSec = 0;
        double startedAtSec = 0;
        LaunchZone zone = LaunchZone.NONE;
    }

    protected ScoringSession scoring = new ScoringSession();

    /** Called once when robot arrives at the scoring  pose. */
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

    /** Call every loop while scoring.active. Returns true when done (hit count or times out). */
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

    /** Optional: force stop (if pathing needs to force stop early). */
    protected void stopScoring() {
        scoring.active = false;
        launchZone = LaunchZone.NONE;
        scoringState = scoreState.EMPTY;
    }
    // ****** Led Controller
    public void LEDDriver()
    {
        if(targetVelocity == 0){decBot.LEDCon(5);}
        else {
            if (leftInGateStatus && rightInGateStatus) {
                decBot.LEDCon(4);
            } else {
                decBot.LEDCon(1);
            }
        }
    }

}