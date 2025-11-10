package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;

import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.FIRING;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.IDLE;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.PREFIRE;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.START_DELAY;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.WAITING_ON_DELAY;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.WAITING_ON_SPEED;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.WAITING_ON_FEED;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;

/**
 * Re written Auto main by Oz Joswick Nov 20 2025
 * Defines functions to be used in other classes
 * When creating object for class enter target velocity for for flywheels(Near 890: far 1093), and shots to be fired
 * Put telemetry out in all main loop in individual class before telemetry update.
 * Launch sequence is what actualy fires the balls
 */

public abstract class AutoMain_oz extends OpMode {
    public Timer start_delay_timer;
    public Timer feed_timer;
    public Timer Ball_delay_timer;
    public Timer max_time_timer;

    protected Double startDelay = 0.0;
    protected double MaxTimePark = 0.0;

    protected float targetVelocity = 0;
    protected int shotsToFire = 0;
    protected int shotsFired = 0;

    protected float feedPerBallMs = 700;
    protected double minTimeBetweenShoots = 2.0;
    protected double flyWheelErrorPercent = 0.05;

    protected boolean startDelayStarted = false;
    protected boolean fireDelayStarted = false;

    protected DecodeBot decBot = new DecodeBot();

    protected enum FiringStates {START_DELAY, IDLE, PREFIRE, FIRING, WAITING_ON_DELAY, WAITING_ON_SPEED, WAITING_ON_FEED}
    protected FiringStates currentState = START_DELAY;

    public AutoMain_oz(){
        start_delay_timer = new Timer();
        feed_timer = new Timer();
        Ball_delay_timer = new Timer();
        max_time_timer = new Timer();
    }

    @Override
    public void init_loop() {
    }

    protected void powerUpFlyWheels(){
        decBot.flylaunch(targetVelocity);
    }

    protected Boolean areFlyAtSpeed() {
        double errorMargin = targetVelocity * flyWheelErrorPercent;
        double lowerBound = targetVelocity - errorMargin;
        double upperBound = targetVelocity + errorMargin;
        double leftVelocity = decBot.leftFlyWheel.getVelocity();
        double rightVelocity = decBot.rightFlyWheel.getVelocity();

        boolean isLeftAtSpeed = leftVelocity >= lowerBound && leftVelocity <= upperBound;
        boolean isRightAtSpeed = rightVelocity >= lowerBound && rightVelocity <= upperBound;

        return isLeftAtSpeed && isRightAtSpeed;
    }

    // The feed logic is now managed by the state machine to avoid blocking the main thread.
    protected void startFeedingBall(){
        decBot.feedArtifact(1);
        feed_timer.resetTimer();
    }

    protected void stopFeedingBall(){
        decBot.feedArtifact(0);
        shotsFired++;
    }

    protected void launchSequence(){
        // CRITICAL FIX: The previously static variables are now non-static instance variables.
        // They must be reset in the concrete class's start() method to prevent state persistence between runs.
        if (max_time_timer.getElapsedTimeSeconds() > MaxTimePark && MaxTimePark > 0.0){
            currentState = IDLE;
        }

        switch(currentState){
            case START_DELAY:
                if (!startDelayStarted){
                    start_delay_timer.resetTimer();
                    startDelayStarted = true;
                }
                if(start_delay_timer.getElapsedTimeSeconds() >= startDelay){
                    currentState = PREFIRE;
                }
                break;

            case PREFIRE:
                powerUpFlyWheels();
                currentState = WAITING_ON_SPEED;
                break;

            case WAITING_ON_SPEED:
                if (areFlyAtSpeed()){
                    if (shotsFired < shotsToFire) {
                        currentState = FIRING;
                    } else {
                        currentState = IDLE;
                    }
                }
                break;

            case FIRING:
                startFeedingBall();
                currentState = WAITING_ON_FEED;
                break;

            // This state manages the non-blocking feed time check
            case WAITING_ON_FEED:
                if(feed_timer.getElapsedTime() >= feedPerBallMs) {
                    stopFeedingBall();
                    Ball_delay_timer.resetTimer();
                    fireDelayStarted = true;
                    currentState = WAITING_ON_DELAY;
                }
                break;

            case WAITING_ON_DELAY:
                if(Ball_delay_timer.getElapsedTimeSeconds() > minTimeBetweenShoots){
                    fireDelayStarted = false;
                    currentState = WAITING_ON_SPEED;
                }
                break;

            case IDLE:
                decBot.feedArtifact(0);
                // The following line was previously redundant and has been removed.
                decBot.flylaunch(0);
                break;
        }
    }

    protected void TelemetryOut(){
        telemetry.addData("State", currentState);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Shots To Fire", shotsToFire);
        telemetry.addData("Shots Fired Already", shotsFired);
    }

}
