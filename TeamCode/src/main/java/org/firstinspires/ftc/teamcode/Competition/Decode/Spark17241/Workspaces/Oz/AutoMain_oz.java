package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;

import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.FIRING;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.IDLE;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.PREFIRE;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.START_DELAY;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.WAITING_ON_DELAY;
import static org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.AutoMain_oz.FiringStates.WAITING_ON_SPEED;

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
    // Timer definitions
    public Timer start_delay_timer;
    public Timer feed_timer;
    public Timer Ball_delay_timer;
    public Timer max_time_timer;

    //delays
    static Double startDelay = 0.0; // delay at start of auto if waiting on other team
    static double MaxTimePark = 0.0; // The last time where robot able to be in shooting area before it leaves to go park

    // definition of varables with values set when object is created.
    static float targetVelocity = 0;
    static int shotsToFire = 0;
    static int shotsFired = 0;

    // variables that control ball feeding
    static float feedPerBallMs = 700; //Time that the feeder wheel spins to feed balls
    static double minTimeBetweenShoots = 2.0; //Absolute minium time between shots if all other factors are good
    static double flyWheelErrorPercent = 0.05; //percent fly wheel can be off and still fire
    static boolean startDelayStarted = false;
    static boolean feedHasFinished = false;
    static boolean fireDelayStarted = false;


    protected DecodeBot decBot = new DecodeBot();

    protected enum FiringStates {START_DELAY, IDLE, PREFIRE, FIRING, WAITING_ON_DELAY, WAITING_ON_SPEED}
    protected FiringStates currentState = START_DELAY;

    public AutoMain_oz(float TargetVelocity, int ShotsToFire){
        start_delay_timer = new Timer();
        feed_timer = new Timer();
        Ball_delay_timer = new Timer();
        max_time_timer = new Timer();

        targetVelocity = TargetVelocity;
        shotsToFire = ShotsToFire;
    }
    protected void powerUpFlyWheels(){
        decBot.flylaunch(targetVelocity);
    }
    protected Boolean areFlyAtSpeed() {
        // Calculate the tolerance value based on the target velocity
        double errorMargin = targetVelocity * flyWheelErrorPercent;
        double lowerBound = targetVelocity - errorMargin;
        double upperBound = targetVelocity + errorMargin;
        double leftVelocity = decBot.leftFlyWheel.getVelocity();
        double rightVelocity = decBot.rightFlyWheel.getVelocity();

        // Check if both flywheels are within the acceptable range
        boolean isLeftAtSpeed = leftVelocity >= lowerBound && leftVelocity <= upperBound;
        boolean isRightAtSpeed = rightVelocity >= lowerBound && rightVelocity <= upperBound;

        if(isLeftAtSpeed && isRightAtSpeed) {
            return true;
        } else {
            return false;
        }
    }
    protected void feedBall(){
        feedHasFinished = false;
        feed_timer.resetTimer();
        while(feed_timer.getElapsedTime() < feedPerBallMs){
            decBot.feedArtifact(1);
        }
            decBot.feedArtifact(0);
        feedHasFinished = true;
    }
    protected void launchSequence(){
        if (max_time_timer.getElapsedTimeSeconds() <= MaxTimePark){ // checks if time to park has happened if so cancel next iteration of launch sequence
            switch(currentState){
                case START_DELAY:
                    if (!startDelayStarted){
                        start_delay_timer.resetTimer();
                        startDelayStarted = true;
                    }
                    if(start_delay_timer.getElapsedTimeSeconds() < startDelay){
                        currentState = PREFIRE;
                    }
                    break;
                case PREFIRE:
                    powerUpFlyWheels();
                    currentState = WAITING_ON_SPEED;
                    break;
                case WAITING_ON_SPEED:
                    if (areFlyAtSpeed()){
                        currentState = FIRING;
                    }
                    break;
                case FIRING:
                    if (shotsFired <= shotsToFire){
                        feedBall();
                        if(feedHasFinished) {
                            feedHasFinished = false;
                            currentState = WAITING_ON_DELAY;
                        }
                    }
                    else{
                        currentState = IDLE;
                    }
                    break;
                case WAITING_ON_DELAY:
                    if (!fireDelayStarted){
                        Ball_delay_timer.resetTimer();
                        fireDelayStarted = true;
                    }
                    if(Ball_delay_timer.getElapsedTimeSeconds() > minTimeBetweenShoots){
                        currentState = WAITING_ON_SPEED;
                    }
                    break;
                case IDLE:
                    decBot.feedArtifact(0);//already happens but double check

                    decBot.feedArtifact(0);
                    break;
            }

        }
    }

    protected void TelemetryOut(){
        telemetry.addData("state", currentState);
        telemetry.addData("target Velocity", targetVelocity);
        telemetry.addData("shots to fire", shotsToFire);
        telemetry.addData("shots fired already", shotsFired);
    }

}