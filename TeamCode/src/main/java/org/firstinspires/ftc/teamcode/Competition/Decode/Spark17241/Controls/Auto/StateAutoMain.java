package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.StateDecodeBot;


//Auto main class writen by Oz Joswick
//Contains general functions for individulized autos based on decode new robot
//contains general varable that apply to all autos based on decode new robot

// no init, start, or loop called in this called in individual autos

// Limlight to be kept in individaul code files as must remember to init diffrent aspects
public abstract class StateAutoMain extends OpMode {
    public com.pedropathing.util.Timer pathTimer, opmodeTimer;

    protected double target_velocity = 0.0;
    protected double gateTolerance = 20.0;

    protected int shotsFired = 0; // will be reset after each lanch


    public StateDecodeBot decBot = new StateDecodeBot();


    //----------------- NORMAL POSES ---------------------------

    //------------Blue----------------

    public final Pose BlueFarStartPose = new Pose(44, 8, Math.toRadians(90)); // Left corner of small triangle
    public final Pose BlueMidShootPose = new Pose(59, 81, Math.toRadians(133));// Blue goal scoring pose
    public final Pose BlueFarShootPose = new Pose(60, 20, Math.toRadians(110));// Blue goal scoring pose from small triangle
    public final Pose BlueFarParkPose = new Pose(43, 12, Math.toRadians(0)); // Blue Home (park)

    // Spike closest to human
    public final Pose BlueSpikeAInsidePose = new Pose(48, 36, Math.toRadians(180));
    public final Pose BlueSpikeAOutsidePose = new Pose(16, 36, Math.toRadians(180));

    // Middle spike
    public final Pose BlueSpikeBInsidePose = new Pose(48, 61, Math.toRadians(180));;
    public final Pose BlueSpikeBOutsidePose = new Pose(16, 61, Math.toRadians(180));

    //Farthest Spike
    public final Pose BlueSpikeCInsidePose = new Pose(97, 85, Math.toRadians(0));
    public final Pose BlueSpikeCOutsidePose = new Pose(128, 85, Math.toRadians(0));
    //----------------------------------------------------------

    //------------Red-----------------
    public final Pose RedFarStartPose =  new Pose(100, 8, Math.toRadians(90));

    public final Pose RedMidShootPose = new Pose(85, 81, Math.toRadians(45));// Red goal scoring pose
    public final Pose RedFarShootPose =  new Pose(84, 20, Math.toRadians(64)); // Red goal scoring pose from small triangle
    public final Pose RedFarParkPose = new Pose(101, 12, Math.toRadians(90)); // Red Home (park)

    public final Pose RedSpikeAInsidePose = new Pose(48, 36, Math.toRadians(180)); // closest to human 23
    public final Pose RedSpikeAOutsidePose = new Pose(16, 36, Math.toRadians(180));

    public final Pose RedSpikeBInsidePose = new Pose(48, 61, Math.toRadians(180)); // secount clostest 22
    public final Pose RedSpikeBOutsidePose = new Pose(16, 61, Math.toRadians(180));

    public final Pose RedSpikeCInsidePose = new Pose(48, 85, Math.toRadians(180)); // third closest 21
    public final Pose RedSpikeCOutsidePose = new Pose(16, 85, Math.toRadians(180));



    /* Velocity drop function and vars
    protected double min_velocity_drop = 50; // threshold for detecting ball contact
    protected List<Double> previousShotVelocityL = new ArrayList<>();
    public boolean velocityDrop() {
        // Track most recent measurements
        double currentVelocity = getCurrentVelocity();
        previousShotVelocityL.add(currentVelocity);

        // Maintain window of last 50 readings
        if (previousShotVelocityL.size() > 50) {
            previousShotVelocityL.remove(0);
        }

        // Detect if significant drop occurred
        double maxRecentVelocity = previousShotVelocityL.stream()
                .mapToDouble(v -> v)
                .max()
                .orElse(currentVelocity);

        double velocityDrop = maxRecentVelocity - currentVelocity;
        return velocityDrop >= min_velocity_drop;
    }*/


    public double getCurrentVelocity() {
        // Averages front and back motor velocity
        return (decBot.launchFrontMotor.getVelocity() + decBot.launchBackMotor.getVelocity()) / 2.0;
    }
    public boolean LaunchWheelsInGate(double selected_speed, double tolerance){
            double upper_tolerance = Math.max(0, selected_speed + tolerance);
            double lower_tolerance = Math.max(0, selected_speed - tolerance);
            double currentVelocity = getCurrentVelocity();
            return currentVelocity >= lower_tolerance && currentVelocity <= upper_tolerance;
    }


    public void prepareForLaunch(){  // while this only has one line of code if prepare condition change use
        decBot.flylaunch(target_velocity);
    }


    public boolean burnerLaunch(double velocity, double currentTime, double startTime){
        decBot.flylaunch(velocity);
        if(currentTime - 5000 > startTime) {
            return true;
        }
        return false;
    }


    /*
    public enum firing {beginFeed, waiting, stopFeed}
    public firing FiringState = firing.beginFeed;

    public boolean shotFiredCondition(){
        return false;
    }


    public void launchOneBall(){
        switch (FiringState){
            case beginFeed:
                if (LaunchWheelsInGate(target_velocity, gateTolerance)){
                    decBot.beginFeed();
                }
                break;
            case waiting:  // current system used to detect if ball has been fired could be changed
                if (shotFiredCondition()){
                    FiringState = firing.stopFeed;
                }
                break;
            case stopFeed:
                shotsFired ++;
                decBot.beginFeed();
                break;
        }
    }

    */
    protected float fireDelay = 4.0f;
    protected float launchStartTime;
    protected boolean hasStartedPrepare = false;
    protected boolean hasStartedLaunch = false;
    public boolean LaunchBalls(double target){// main launch method
        target_velocity = target;
        if (!hasStartedPrepare) {
            hasStartedPrepare = true;
            prepareForLaunch();
        }
        if (LaunchWheelsInGate(target_velocity, gateTolerance) && (!hasStartedLaunch)){
            hasStartedLaunch = true;
            launchStartTime = opmodeTimer.getElapsedTime();
            decBot.beginFeed();
        }
        if ((opmodeTimer.getElapsedTime() - fireDelay >= launchStartTime) && hasStartedLaunch){
            hasStartedPrepare = false;
            hasStartedLaunch = false;
            decBot.flylaunch(0);
            return true;
        }
        return false;
        }
    public void AutoMainTelemetry(){
        telemetry.addData("target velocity", target_velocity);
        telemetry.addData("Gate Tolerance", gateTolerance);
        telemetry.addData("shots fired", shotsFired);
        telemetry.addData("hasStartedLaunch", hasStartedLaunch);
    }

}
