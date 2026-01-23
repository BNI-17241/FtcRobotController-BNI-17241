package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.StateDecodeBot;

import java.util.ArrayList;
import java.util.List;

//Auto main class writen by Oz Joswick
//Contains general functions for individulized autos based on decode new robot
//contains general varable that apply to all autos based on decode new robot

// no init, start, or loop called in this called in individual autos

// Limlight to be kept in individaul code files as must remember to init diffrent aspects
public abstract class AutoMainNew extends OpMode {
    protected double target_velocity = 0.0;
    protected double gateTolerance = 20.0;

    protected int shotsFired = 0; // will be reset after each lanch


    public StateDecodeBot decBot = new StateDecodeBot();


    //----------------- NORMAL POSES ---------------------------

    //------------Blue----------------

    public final Pose BlueFarStartPose = new Pose(44, 8, Math.toRadians(90)); // Left corner of small triangle
    public final Pose BlueMidShootPose = new Pose(59, 81, Math.toRadians(133));// Blue goal scoring pose
    public final Pose BlueFarShootPose = new Pose(60, 20, Math.toRadians(110));// Blue goal scoring pose from small triangle
    public final Pose BlueFarParkPose = new Pose(43, 12, Math.toRadians(90)); // Blue Home (park)

    // Spike closest to human
    public final Pose BlueSpikeAInsidePose = new Pose(97, 36, Math.toRadians(0));
    public final Pose BlueSpikeAOutsidePose = new Pose(128, 36, Math.toRadians(0));

    // Middle spike
    public final Pose BlueSpikeBInsidePose = new Pose(97, 61, Math.toRadians(0));
    public final Pose BlueSpikeBOutsidePose = new Pose(128, 61, Math.toRadians(0));

    //Farthest Spike
    public final Pose BlueSpikeCInsidePose = new Pose(97, 85, Math.toRadians(0));
    public final Pose BlueSpikeCOutsidePose = new Pose(128, 85, Math.toRadians(0));
    //----------------------------------------------------------


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
    }


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

    public enum firing {beginFeed, waiting, stopFeed}
    public firing FiringState = firing.beginFeed;
    public void launchOneBall(){
        switch (FiringState){
            case beginFeed:
                decBot.beginFeed();
            case waiting:  // current system used to detect if ball has been fired could be changed
                if (velocityDrop()){
                    FiringState = firing.stopFeed;
            }
            case stopFeed:
                shotsFired ++;
                decBot.stopFeed();


        }
    }
    public boolean LaunchBalls(int num){// main launch method
        prepareForLaunch();
        if (shotsFired <= num){
            if (LaunchWheelsInGate(target_velocity, gateTolerance)) {
                launchOneBall();
            }
        }
        else {
            FiringState = firing.beginFeed;
            decBot.flylaunch(0);
            shotsFired = 0;
            return true;
        }
        return false;
        }
    public void AutoMainTelemetry(){
        telemetry.addData("target velocity", target_velocity);
        telemetry.addData("Gate Tolerance", gateTolerance);
        telemetry.addData("shots fired", shotsFired);
        telemetry.addData("Firing state", FiringState);
    }

}
