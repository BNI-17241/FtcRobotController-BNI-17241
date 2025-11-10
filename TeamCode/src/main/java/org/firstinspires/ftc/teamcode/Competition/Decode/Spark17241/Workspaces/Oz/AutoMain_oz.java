package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;

import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.FIRING;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.IDLE;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.PREFIRE;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.START_DELAY;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.WAITING_ON_DELAY;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.WAITING_ON_SPEED;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.WAITING_ON_FEED; // I DO NOT REMEMBER WRITING THIS AND APPARENTLY IT IS HERE

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
    // Timer definitions (Non-static, as they track state per run)
    public Timer start_delay_timer;
    public Timer feed_timer;
    public Timer Ball_delay_timer;
    public Timer max_time_timer;

    // delays (Non-static)
    // These values should ideally be set in init/constructor and not changed at runtime,
    // but made non-static to ensure they reset with each Opmode instance.
    protected Double startDelay = 0.0; // delay at start of auto if waiting on other team
    protected double MaxTimePark = 0.0; // The last time where robot able to be in shooting area before it leaves to go park

    // definition of varables with values set when object is created. (Non-static)
    protected float targetVelocity;
    protected int shotsToFire;
    protected int shotsFired; // Initialize in constructor

    // variables that control ball feeding (Non-static)
    protected float feedPerBallMs = 700; //Time that the feeder wheel spins to feed balls
    protected double minTimeBetweenShoots = 2.0; //Absolute minium time between shots if all other factors are good
    protected double flyWheelErrorPercent = 0.05; //percent fly wheel can be off and still fire

    // State management flags (Non-static, reset per run)
    protected boolean startDelayStarted; // Initialize in constructor
    // Removed feedHasFinished as the state machine manages this now
    protected boolean fireDelayStarted; // Initialize in constructor


    protected DecodeBot decBot = new DecodeBot();

    // Added WAITING_ON_FEED state to manage non-blocking feeding
    protected enum FiringStates {START_DELAY, IDLE, PREFIRE, FIRING, WAITING_ON_DELAY, WAITING_ON_SPEED, WAITING_ON_FEED}
    protected FiringStates currentState; // Initialize in constructor

    public AutoMain_oz(float TargetVelocity, int ShotsToFire){
        // Initialize Timers (Timers handle their own internal state reset)
        start_delay_timer = new Timer();
        feed_timer = new Timer();
        Ball_delay_timer = new Timer();
        max_time_timer = new Timer();

        // Initialize user-defined parameters
        targetVelocity = TargetVelocity;
        shotsToFire = ShotsToFire;

        // CRITICAL FIX 1: Ensure all flags/counters are reset for a fresh run (changed from static to non-static variables).
        shotsFired = 0;
        startDelayStarted = false;
        fireDelayStarted = false;
        currentState = START_DELAY;
    }

    // The init() method is a good place to start timers that need to run from the moment init is pressed
    @Override
    public void init() {
        // Assume decBot.init() is called elsewhere or in the specific OpMode implementation
        max_time_timer.resetTimer();
        // Telemetry updates happen in the calling OpMode's loop()
    }

    protected void powerUpFlyWheels(){
        // Note: decBot.flylaunch assumes the robot object handles setting motor modes/power correctly
        decBot.flylaunch(targetVelocity);
    }

    protected Boolean areFlyAtSpeed() {
        // Calculate the tolerance value based on the target velocity
        double errorMargin = targetVelocity * flyWheelErrorPercent;
        double lowerBound = targetVelocity - errorMargin;
        double upperBound = targetVelocity + errorMargin;

        // Note: These methods assume the DecodeBot class properly interfaces with FTC hardware
        double leftVelocity = decBot.leftFlyWheel.getVelocity();
        double rightVelocity = decBot.rightFlyWheel.getVelocity();

        // Check if both flywheels are within the acceptable range
        boolean isLeftAtSpeed = leftVelocity >= lowerBound && leftVelocity <= upperBound;
        boolean isRightAtSpeed = rightVelocity >= lowerBound && rightVelocity <= upperBound;

        return isLeftAtSpeed && isRightAtSpeed;
    }

    /*
     * CRITICAL FIX 2: The feedBall logic is moved into the state machine to avoid blocking the main OpMode thread
     * with a 'while' loop, which is illegal in standard FTC OpModes.
     */
    protected void startFeedingBall(){
        decBot.feedArtifact(1); // Turn on feeder
        feed_timer.resetTimer(); // Start tracking how long the feeder has been on
    }

    protected void stopFeedingBall(){
        decBot.feedArtifact(0); // Turn off feeder
        shotsFired++; // Increment the counter when a shot sequence is complete
    }

    protected void launchSequence(){
        // checks if time to park has happened. If so, move to IDLE state.
        if (max_time_timer.getElapsedTimeSeconds() > MaxTimePark && MaxTimePark > 0.0){
            currentState = IDLE;
        }

        // This switch statement runs one iteration of the state machine every time loop() is called
        switch(currentState){
            case START_DELAY:
                if (!startDelayStarted){
                    start_delay_timer.resetTimer();
                    startDelayStarted = true;
                }
                // Transition condition: Check if the delay time has passed
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
                    // Check if we still have shots to fire before transitioning to firing
                    if (shotsFired < shotsToFire) {
                        currentState = FIRING;
                    } else {
                        currentState = IDLE; // All shots done, stop
                    }
                }
                break;

            case FIRING:
                // Start the feeding process, but don't block here
                startFeedingBall();
                currentState = WAITING_ON_FEED; // Immediately move to a waiting state
                break;

            // NEW STATE: Wait for the correct feed time duration
            case WAITING_ON_FEED:
                if(feed_timer.getElapsedTime() >= feedPerBallMs) {
                    stopFeedingBall(); // Turn off feeder and increment shot counter

                    // After feeding, we immediately start the ball delay timer and transition
                    Ball_delay_timer.resetTimer();
                    fireDelayStarted = true; // Flag indicates timer is running
                    currentState = WAITING_ON_DELAY;
                }
                break;

            case WAITING_ON_DELAY:
                // Check if the required delay between shots has passed
                if(Ball_delay_timer.getElapsedTimeSeconds() > minTimeBetweenShoots){
                    // Reset flag for next cycle
                    fireDelayStarted = false;
                    currentState = WAITING_ON_SPEED; // Go back to check speed for the next shot
                }
                break;

            case IDLE:
                // MINOR FIX 3: Removed redundant decBot.feedArtifact(0) line.
                decBot.feedArtifact(0);
                decBot.flylaunch(0); // Assuming flylaunch(0) stops the flywheels
                break;
        }
    }

    protected void TelemetryOut(){
        telemetry.addData("State", currentState);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Shots To Fire", shotsToFire);
        telemetry.addData("Shots Fired Already", shotsFired);
        telemetry.addData("Left Flywheel Vel", decBot.leftFlyWheel.getVelocity());
        telemetry.addData("Right Flywheel Vel", decBot.rightFlyWheel.getVelocity());
    }

}
