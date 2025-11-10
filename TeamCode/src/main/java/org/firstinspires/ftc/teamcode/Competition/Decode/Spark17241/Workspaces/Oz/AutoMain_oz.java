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
    public Timer first_shot_timer;

    // These variables store settings for a specific autonomous run and are not shared between different runs.
    protected Double startDelay = 0.0; // delay at start of auto if waiting on other team
    protected double MaxTimePark = 0.0; // The last time where robot able to be in shooting area before it leaves to go park

    protected float targetVelocity = 0;
    protected int shotsToFire = 0;
    protected int shotsFired = 0;

    // variables that control ball feeding
    protected float feedPerBallMs = 700; //Time that the feeder wheel spins to feed balls
    protected double minTimeBetweenShoots = 2.0; //Absolute minium time between shots if all other factors are good
    protected double flyWheelErrorPercent = 0.05; //percent fly wheel can be off and still fire
    protected double firstShotMinTimme = 5.5;

    // Flags to track the progress of the firing process
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
        first_shot_timer = new Timer();
    }

    @Override
    public void init_loop() {
    }

    // Turns the flywheel motors on
    protected void powerUpFlyWheels(){
        decBot.flylaunch(targetVelocity);
    }

    // Checks if both flywheels are spinning within the acceptable speed range
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
            currentState = IDLE;
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
                    currentState = PREFIRE;
                }
                break;

            case PREFIRE:
                // Turn on flywheels and move to wait for speed
                powerUpFlyWheels();
                currentState = WAITING_ON_SPEED;
                break;

            case WAITING_ON_SPEED:
                // Check speed and either start firing or finish the sequence
                if (areFlyAtSpeed()){
                    if (shotsFired < shotsToFire) {
                        currentState = FIRING;
                    } else {
                        currentState = IDLE;
                    }
                }
                break;

            case FIRING:
                // Begin the feeding process and move immediately to the waiting state
                if (first_shot_timer.getElapsedTimeSeconds() >= firstShotMinTimme){
                    startFeedingBall();
                    currentState = WAITING_ON_FEED;
                }

                break;

            case WAITING_ON_FEED:
                // Wait until the ball has been fed for the correct amount of time
                if(feed_timer.getElapsedTime() >= feedPerBallMs) {
                    stopFeedingBall();
                    Ball_delay_timer.resetTimer();
                    fireDelayStarted = true;
                    currentState = WAITING_ON_DELAY;
                }
                break;

            case WAITING_ON_DELAY:
                // Wait for the minimum time between shots before checking speed again
                if(Ball_delay_timer.getElapsedTimeSeconds() > minTimeBetweenShoots){
                    fireDelayStarted = false;
                    currentState = WAITING_ON_SPEED;
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
