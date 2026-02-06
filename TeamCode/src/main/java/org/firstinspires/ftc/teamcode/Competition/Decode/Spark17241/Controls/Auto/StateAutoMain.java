package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.StateDecodeBot;

import java.util.List;


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

    //Power threshold for moving
    protected double powerThreshold;



    //----------------- NORMAL POSES ---------------------------

    //------------Blue----------------

    public final Pose BlueGoalStartPose = new Pose(22, 122, Math.toRadians(135));
    public final Pose BlueNearParkPose = new Pose(50, 130, Math.toRadians(270));

    public final Pose BlueFarStartPose = new Pose(44, 8, Math.toRadians(90)); // Left corner of small triangle

    public final Pose BlueMidShootPose = new Pose(57, 83, Math.toRadians(13));// Blue goal scoring pose

    public final Pose BlueFarShootPose = new Pose(60, 20, Math.toRadians(108));// Blue goal scoring pose from small triangle
    public final Pose BlueFarParkPose = new Pose(43, 12, Math.toRadians(90)); // Blue Home (park)

    // Spike closest to human
    public final Pose BlueSpikeAInsidePose = new Pose(48, 33, Math.toRadians(180)); //31
    public final Pose BlueSpikeAOutsidePose = new Pose(17, 33, Math.toRadians(180)); //31

    // Middle spike
    public final Pose BlueSpikeBInsidePose = new Pose(48, 56, Math.toRadians(180));//57
    public final Pose BlueSpikeBOutsidePose = new Pose(17, 56, Math.toRadians(180));//57

    //Farthest Spike
    public final Pose BlueSpikeCInsidePose = new Pose(48, 81, Math.toRadians(180));
    public final Pose BlueSpikeCOutsidePose = new Pose(17, 81, Math.toRadians(180));
    //----------------------------------------------------------

    //------------Red-----------------
    public final Pose RedFarStartPose =  new Pose(100, 8, Math.toRadians(90));
    public final Pose RedMidShootPose = new Pose(85, 81, Math.toRadians(45));// Red goal scoring pose

    public final Pose RedFarShootPose =  new Pose(84, 20, Math.toRadians(64)); // Red goal scoring pose from small triangle

    public final Pose RedFarParkPose = new Pose(101, 12, Math.toRadians(90)); // Red Home (park)

    public final Pose RedSpikeAInsidePose = new Pose(105, 32, Math.toRadians(0)); // closest to human 23
    public final Pose RedSpikeAOutsidePose = new Pose(124, 32, Math.toRadians(0));

    public final Pose RedSpikeBInsidePose = new Pose(105, 57, Math.toRadians(0)); // secount clostest 22
    public final Pose RedSpikeBOutsidePose = new Pose(124, 57, Math.toRadians(0));

    public final Pose RedSpikeCInsidePose = new Pose(105, 81, Math.toRadians(0)); // third closest 21
    public final Pose RedSpikeCOutsidePose = new Pose(124, 81, Math.toRadians(0));



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


    protected Limelight3A limelight;
    //Limelight Cam data
    protected LLResult result;

    public void limelightInit(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    //Auto Correction
    public void limeLightData() {
        result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
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


    public boolean burnerLaunch(double currentTime, double startTime, double variance, double velocity){
        if(currentTime - 5000 > startTime) {
            decBot.stopFeed();
            return true;
        }
        if(LaunchWheelsInGate(velocity, variance)){
            decBot.beginFeed();
        }
        else{
            decBot.stopFeed();
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

    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }


    public void autoTarget(double offset) {
        if (result == null || !result.isValid()) {
            telemetry.addData("AutoTarget", "No valid Limelight result");
            return;
        }
        else{
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                //Don't rotate for motif tags
                if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                Pose3D tagInCam = fr.getTargetPoseCameraSpace();     // pose of tag in CAMERA space
                double x = tagInCam.getPosition().x;  // right (+)
                double y = tagInCam.getPosition().y;  // down (+)

                double forwardMeters = tagInCam.getPosition().z;  // forward/out of camera (+)
                double rangeMeters = Math.sqrt(x * x + y * y + forwardMeters * forwardMeters);// 3D range

                telemetry.addLine("-------------------------------------");
                telemetry.addData("Tag ID", fr.getFiducialId());
                telemetry.addData("Forward (m)", "%.3f", forwardMeters);
                telemetry.addData("Range (m)", "%.3f", rangeMeters);
                telemetry.addData("Yaw", fr.getTargetXDegrees());
                telemetry.addLine("-------------------------------------");
                double tagXDegrees;




                    //Check for tags and get X degrees
                    tagXDegrees = fr.getFiducialId() == 20 ? fr.getTargetXDegrees() : 0;
                    tagXDegrees = fr.getFiducialId() == 24 ? fr.getTargetXDegrees() : tagXDegrees;

                    // --- Proportional Drive Control parameters  ---
                    double kP = 0.03;             // Proportional gain for turning and oscillation
                    double maxTurnSpeed = .15;   // Max turn power
                    double minTurnSpeed = .1;  // Minimum turn power to overcome friction
                    double tolerance = 1;      // Deadband in degrees that controls oscillation

                    //tagXDegrees += offset;

                    // If we’re close enough, stop and don’t oscillate
                    if (Math.abs(tagXDegrees) < tolerance) {
                        decBot.frontLeftMotor.setPower(0);
                        decBot.rearLeftMotor.setPower(0);
                        decBot.frontRightMotor.setPower(0);
                        decBot.rearRightMotor.setPower(0);
                        telemetry.addData("Align", "Aligned! tx=%.2f", tagXDegrees);
                        telemetry.addData("Tag", "ID: %d", fr.getFiducialId());
                        return;
                    }

                    // Proportional turning power
                    double power = kP * tagXDegrees;

                    // Clip to max speed
                    if (power > maxTurnSpeed) power = maxTurnSpeed;
                    if (power < -maxTurnSpeed) power = -maxTurnSpeed;

                    // Enforce a minimum power when we’re still outside tolerance
                    if (power > 0 && Math.abs(power) < minTurnSpeed) power = minTurnSpeed;
                    if (power < 0 && Math.abs(power) < minTurnSpeed) power = -minTurnSpeed;

                    // Map sign so that:
                    //  tx < 0 (tag left) so robot turns left (fl -, fr +)
                    //  tx > 0 (tag right) so robots turns right (fl +, fr -)
                    double frontLeft = power;
                    double frontRight = -power;
                    double rearLeft = power;
                    double rearRight = -power;

                    // Set motor powers
                    setMotorPower(decBot.frontLeftMotor, frontLeft, powerThreshold, 1.0);
                    setMotorPower(decBot.frontRightMotor, frontRight, powerThreshold, 1.0);
                    setMotorPower(decBot.rearLeftMotor, rearLeft, powerThreshold, 1.0);
                    setMotorPower(decBot.rearRightMotor, rearRight, powerThreshold, 1.0);

                    telemetry.addData("Align", "tx=%.2f, power=%.2f", tagXDegrees, power);

                }
            }
        }
    }

}


