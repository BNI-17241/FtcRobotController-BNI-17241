package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;

/**
 * Rewritten Auto Main by Oz Joswick — Nov 20, 2025
 * Defines functions to be used in other classes.
 * When creating an object for this class, enter target velocities for flywheels
 * (Near: ~890 | Far: ~1093) and number of shots to be fired.
 *
 * Telemetry outputs should be handled in the loop() of the specific OpMode
 * before telemetry.update().
 * The launch sequence handles the ball firing process.
 */

public abstract class AutoMain_NewAndrew extends OpMode {
    public Timer start_delay_timer;
    public Timer feed_timer;
    public Timer Ball_delay_timer;
    public Timer max_time_timer;
    public Timer first_shot_timer;
    public Timer auto_Timer;

    // Configurable autonomous parameters
    protected double startDelay = 0.0;       // Delay before auto starts
    protected double MaxTimePark = 30.0;      // Last allowed shooting time before parking
    protected float targetVelocity = 0;
    protected float targetVelocityTwo = 0;
    protected float targetVelocityThree = 0;

    protected int shotsToFire = 0;
    protected int shotsFired = 0;
    protected int feedNum = 1;

    // Feed timing (milliseconds)
    protected float firstFeedPerBall = 700;//700
    protected float secondFeedPerBall = 250;//250
    protected float thirdFeedPerBall = 600;//600

    protected double minTimeBetweenShoots = 2.0;  // Seconds
    protected double flyWheelErrorPercent = 0.05;
    protected double firstShotMinTime = 5.5;

    // State flags
    protected boolean startDelayStarted = false;
    protected boolean fireDelayStarted = false;

    protected DecodeBot decBot = new DecodeBot();

    protected enum FiringStates {
        START_DELAY, IDLE, PREFIRE, FIRING, WAITING_ON_DELAY, WAITING_ON_SPEED, WAITING_ON_FEED
    }

    protected FiringStates currentState = FiringStates.START_DELAY;

    public AutoMain_NewAndrew() {
        start_delay_timer = new Timer();
        feed_timer = new Timer();
        Ball_delay_timer = new Timer();
        max_time_timer = new Timer();
        first_shot_timer = new Timer();

        auto_Timer = new Timer();
    }


    protected void powerUpFlyWheels() {
        if (shotsFired == 0) {
            decBot.flylaunch(targetVelocity);
        } else if (shotsFired == 1) {
            decBot.flylaunch(targetVelocityTwo);
        } else {
            decBot.flylaunch(targetVelocityThree);
        }
    }


    protected boolean areFlyAtSpeed() {
        double target = (shotsFired == 0) ? targetVelocity :
                (shotsFired == 1) ? targetVelocityTwo : targetVelocityThree;

        double errorMargin = target * flyWheelErrorPercent;
        double lowerBound = target - errorMargin;
        double upperBound = target + errorMargin;

        double leftVelocity = decBot.leftFlyWheel.getVelocity();
        double rightVelocity = decBot.rightFlyWheel.getVelocity();

        boolean isLeftAtSpeed = leftVelocity >= lowerBound && leftVelocity <= upperBound;
        boolean isRightAtSpeed = rightVelocity >= lowerBound && rightVelocity <= upperBound;

        return isLeftAtSpeed && isRightAtSpeed;
    }


    protected void startFeedingBall() {
        decBot.feedArtifact(1);
        feed_timer.resetTimer();
    }


    protected void stopFeedingBall() {
        decBot.feedArtifact(0);
        shotsFired++;
    }


    protected void launchSequence() {
        // Stop firing if max allowed time is reached
        if (MaxTimePark > 0.0 && max_time_timer.getElapsedTimeSeconds() > MaxTimePark) {
            currentState = FiringStates.IDLE;
        }

        switch (currentState) {
            case START_DELAY:
                if (!startDelayStarted) {
                    start_delay_timer.resetTimer();
                    startDelayStarted = true;
                }
                if (start_delay_timer.getElapsedTimeSeconds() >= startDelay) {
                    currentState = FiringStates.PREFIRE;
                }
                break;

            case PREFIRE:
                powerUpFlyWheels();
                if (shotsFired == 0) {
                    currentState = FiringStates.WAITING_ON_SPEED;
                    first_shot_timer.resetTimer();
                } else {
                    currentState = FiringStates.WAITING_ON_SPEED;//-------------------------------I changed this from WAITING_ON_DELAY
                }
                break;

            case WAITING_ON_SPEED:
                if (areFlyAtSpeed()) {
                    if (shotsFired < shotsToFire) {
                        currentState = FiringStates.FIRING;
                    } else {
                        currentState = FiringStates.IDLE;
                    }
                }
                break;

            case FIRING:
                if (first_shot_timer.getElapsedTimeSeconds() >= firstShotMinTime) {
                    startFeedingBall();
                    currentState = FiringStates.WAITING_ON_FEED;
                }
                break;

            case WAITING_ON_FEED:
                double elapsedFeedTime = feed_timer.getElapsedTimeSeconds() * 1000; // Convert sec → ms

                if (feedNum == 1 && elapsedFeedTime >= firstFeedPerBall) {
                    stopFeedingBall();
                    Ball_delay_timer.resetTimer();
                    feedNum = 2;
                    currentState = FiringStates.WAITING_ON_DELAY;
                } else if (feedNum == 2 && elapsedFeedTime >= secondFeedPerBall) {
                    stopFeedingBall();
                    Ball_delay_timer.resetTimer();
                    feedNum = 3;
                    currentState = FiringStates.WAITING_ON_DELAY;
                } else if (feedNum == 3 && elapsedFeedTime >= thirdFeedPerBall) {
                    stopFeedingBall();
                    Ball_delay_timer.resetTimer();
                    currentState = FiringStates.WAITING_ON_DELAY;
                }
                break;

            case WAITING_ON_DELAY:
                if (Ball_delay_timer.getElapsedTimeSeconds() > minTimeBetweenShoots) {
                    fireDelayStarted = false;
                    if (shotsFired < shotsToFire) {
                        currentState = FiringStates.PREFIRE;
                    } else {
                        currentState = FiringStates.IDLE;
                    }
                }
                break;

            case IDLE:
                decBot.feedArtifact(0);
                decBot.flylaunch(0);
                break;
        }
    }


    protected void TelemetryOut() {
        telemetry.addData("State", currentState);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Shots To Fire", shotsToFire);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Feed Num", feedNum);
        telemetry.addData("Flywheel At Speed", areFlyAtSpeed());
        telemetry.addData("Start Timer: ", start_delay_timer.getElapsedTimeSeconds());
        telemetry.addData("Max Time Timer: ",  max_time_timer.getElapsedTimeSeconds());
        telemetry.addData("First Shot Timer: ", first_shot_timer.getElapsedTimeSeconds());
        telemetry.addData("Auto Timer: ", auto_Timer.getElapsedTimeSeconds());
        telemetry.addData("Ball Delay Timer: ", Ball_delay_timer.getElapsedTimeSeconds());
    }


    public void LEDDriver() {
        if (targetVelocity == 0) {
            decBot.LEDCon(5);
        } else if (areFlyAtSpeed()) {
            decBot.LEDCon(4);
        } else {
            decBot.LEDCon(1);
        }
    }
}
