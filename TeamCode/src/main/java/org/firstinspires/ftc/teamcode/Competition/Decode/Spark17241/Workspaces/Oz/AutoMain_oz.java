package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;


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
    public Timer first_shot_timer;

    // These variables store settings for a specific autonomous run and are not shared between different runs.
    protected Double startDelay = 0.0; // delay at start of auto if waiting on other team
    protected double MaxTimePark = 0.0; // The last time where robot able to be in shooting area before it leaves to go park

    protected float targetVelocity = 0;
    protected float targetVelocityTwo = 0;
    protected float targetVelocityThree = 0;

    protected int shotsToFire = 0;
    protected int shotsFired = 0;
    protected int feedNum = 1;

    // variables that control ball feeding
    protected float firstFeedPerBall = 700; //Time that the feeder wheel spins to feed balls
    protected float secountFeedPerBall = 200; // must add more  times if you want more + edit later code + edit feed num thing idk
    protected float thirdFeedPerBall = 600;

    protected double minTimeBetweenShoots = 2.0; //Absolute minium time between shots if all other factors are good
    protected double flyWheelErrorPercent = 0.05; //percent fly wheel can be off and still fire
    protected double firstShotMinTimme = 5.5;

    // Flags to track the progress of the firing process
    protected boolean startDelayStarted = false;
    protected boolean fireDelayStarted = false;

    protected DecodeBot decBot = new DecodeBot();

    protected enum FiringStates {START_DELAY, IDLE, PREFIRE, FIRING, WAITING_ON_DELAY, WAITING_ON_SPEED, WAITING_ON_FEED}
    protected FiringStates currentState = FiringStates.START_DELAY;

    public AutoMain_oz(){
        start_delay_timer = new Timer();
        feed_timer = new Timer();
        Ball_delay_timer = new Timer();
        max_time_timer = new Timer();
        first_shot_timer = new Timer();
    }

    // Turns the flywheel motors on
    protected void powerUpFlyWheels(){
        if (shotsFired == 0){
            decBot.flylaunch(targetVelocity);
        }
        if (shotsFired == 1){
            decBot.flylaunch(targetVelocityTwo);
        }
        else {
            decBot.flylaunch(targetVelocityThree);
        }
    }

    // Checks if both flywheels are spinning within the acceptable speed range
    protected Boolean areFlyAtSpeed() {
        if (shotsFired == 0){
        double errorMargin = targetVelocity * flyWheelErrorPercent;
        double lowerBound = targetVelocity - errorMargin;
        double upperBound = targetVelocity + errorMargin;
        double leftVelocity = decBot.leftFlyWheel.getVelocity();
        double rightVelocity = decBot.rightFlyWheel.getVelocity();

        boolean isLeftAtSpeed = leftVelocity >= lowerBound && leftVelocity <= upperBound;
        boolean isRightAtSpeed = rightVelocity >= lowerBound && rightVelocity <= upperBound;

        return isLeftAtSpeed && isRightAtSpeed;
        } else if (shotsFired == 1){
            double errorMargin = targetVelocityTwo * flyWheelErrorPercent;
            double lowerBound = targetVelocityTwo - errorMargin;
            double upperBound = targetVelocityTwo + errorMargin;
            double leftVelocity = decBot.leftFlyWheel.getVelocity();
            double rightVelocity = decBot.rightFlyWheel.getVelocity();

            boolean isLeftAtSpeed = leftVelocity >= lowerBound && leftVelocity <= upperBound;
            boolean isRightAtSpeed = rightVelocity >= lowerBound && rightVelocity <= upperBound;

            return isLeftAtSpeed && isRightAtSpeed;
        }
        else{
            double errorMargin = targetVelocityThree * flyWheelErrorPercent;
            double lowerBound = targetVelocityThree - errorMargin;
            double upperBound = targetVelocityThree + errorMargin;
            double leftVelocity = decBot.leftFlyWheel.getVelocity();
            double rightVelocity = decBot.rightFlyWheel.getVelocity();

            boolean isLeftAtSpeed = leftVelocity >= lowerBound && leftVelocity <= upperBound;
            boolean isRightAtSpeed = rightVelocity >= lowerBound && rightVelocity <= upperBound;

            return isLeftAtSpeed && isRightAtSpeed;
        }
    }


    // Starts the feeder motor and the timer for the feeding duration
    protected void startFeedingBall(){
        decBot.feedArtifact(1);
        feed_timer.resetTimer();
    }

    // Stops the feeder motor and counts that one shot has been fired
    protected void stopFeedingBall(){
        decBot.feedArtifact(0);
        shotsFired++;
    }

    // This runs one step of the firing process state machine
    protected void launchSequence(){
        // Check if the overall auto time limit has passed, if so, stop firing.
        if (max_time_timer.getElapsedTimeSeconds() > MaxTimePark && MaxTimePark > 0.0){
            currentState = FiringStates.IDLE;
        }

        // The switch statement moves between states every time 'loop()' runs in the main OpMode
        switch(currentState){
            case START_DELAY:
                // Start the initial delay timer
                if (!startDelayStarted){
                    start_delay_timer.resetTimer();
                    startDelayStarted = true;
                }
                // Transition to the next state when the delay time is over
                if(start_delay_timer.getElapsedTimeSeconds() >= startDelay){
                    currentState = FiringStates.PREFIRE;
                }
                break;

            case PREFIRE:
                // Turn on flywheels and move to wait for speed
                powerUpFlyWheels();
                if (shotsFired == 0){
                    currentState = FiringStates.WAITING_ON_SPEED;
                }
                else{
                    currentState = FiringStates.WAITING_ON_DELAY;
                }

                break;

            case WAITING_ON_SPEED:
                // Check speed and either start firing or finish the sequence
                if (areFlyAtSpeed()){
                    if (shotsFired < shotsToFire) {
                        currentState = FiringStates.FIRING;
                    } else {
                        currentState = FiringStates.IDLE;
                    }
                }
                break;

            case FIRING:
                // Begin the feeding process and move immediately to the waiting state
                if (first_shot_timer.getElapsedTimeSeconds() >= firstShotMinTimme){
                    startFeedingBall();
                    currentState = FiringStates.WAITING_ON_FEED;
                    targetVelocity = targetVelocity - 100;
                }

                break;

            case WAITING_ON_FEED:
                if (feedNum == 1){
                    if(feed_timer.getElapsedTime() >= firstFeedPerBall) {
                        stopFeedingBall();
                        Ball_delay_timer.resetTimer();
                        fireDelayStarted = true;
                        currentState = FiringStates.WAITING_ON_DELAY;
                        feedNum = 2;
                    }
                }else if(feedNum == 2) {
                    if (feed_timer.getElapsedTime() >= secountFeedPerBall) {
                        stopFeedingBall();
                        Ball_delay_timer.resetTimer();
                        fireDelayStarted = true;
                        currentState = FiringStates.WAITING_ON_DELAY;
                        feedNum = 3;
                    }
                }else if (feedNum == 3) {
                    if (feed_timer.getElapsedTime() >= thirdFeedPerBall) {
                        stopFeedingBall();
                        Ball_delay_timer.resetTimer();
                        fireDelayStarted = true;
                        currentState = FiringStates.WAITING_ON_DELAY;
                        feedNum = 3;
                    }
                }
                break;
            case WAITING_ON_DELAY:
                // Wait for the minimum time between shots before checking speed again
                if(Ball_delay_timer.getElapsedTimeSeconds() > minTimeBetweenShoots){
                    fireDelayStarted = false;
                    currentState = FiringStates.PREFIRE;
                }
                break;

            case IDLE:
                // Stop all firing related mechanisms
                decBot.feedArtifact(0);
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

public void LEDDriver()
    {
        if(targetVelocity == 0){decBot.LEDCon(5);}
        else {
            if (areFlyAtSpeed()) {
                decBot.LEDCon(4);
            } else {
                decBot.LEDCon(1);
            }
        }
    }

}
