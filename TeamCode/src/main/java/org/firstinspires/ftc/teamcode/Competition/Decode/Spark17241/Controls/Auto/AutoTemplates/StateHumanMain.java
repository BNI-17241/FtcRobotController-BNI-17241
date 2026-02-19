package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoTemplates;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.StateAutoMain;


public abstract class StateHumanMain extends StateAutoMain {

    //--------------Config for paths-----------------------
    //Start Pose
    public Pose StartingPose ;
    //Shoot Pose
    public Pose ShootingPose;
    //Park Pose
    public Pose ParkingPose;

    //Delay before initial movement (ms)
    public float startDelay;

    public double variance;

    public double xAutoOffset;

    //Offset for spike to fire paths
    public double pathOffset;

    //When to go to park as failsafe (0-30 seconds from start, recommended 25)
    public double maxTimeBreakout;
    //-----------------------------------------------------

    /**
     * Pedro Pathing Variables, Poses, Paths & States
     */



    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    //How many spikes have been intook
    public int spikesTaken;

    //Time reference for burnerlaunch function
    public double startFireTime;

    //path for the moveToPoint case used in spike navigation
    public PathChain moveToPointChain;

    //Help view elapsed time on wait cases
    public float delayStartTime = 0;

    public Path scorePreload;

    protected PathChain start_to_fire;
    protected PathChain fire_to_spike1;
    protected PathChain fire_to_spike4;
    protected PathChain spike1_traversal;
    protected PathChain spike4_traversal;
    protected PathChain spike1_to_fire;
    protected PathChain spike4_to_fire;
    protected PathChain fire_to_park;

    //Base target velocity
    public double targetVelocity;

    //Base intake spin speed
    public double intakeSpeed;

    //Base motor power limit while intaking (0-1)
    public double intakeMoveSpeed;

    //set up simple states
    public enum pathingState {
        STARTDELAY, START, INTAKESPIKES, PARK, END, MOVETOPOINT, RETURNMOVETOPOINT, FIREANDRETURNSTATE,
        PREFIRE, INTAKETOPOINT, TAKESPIKEONE, TAKESPIKEFOUR, AUTOTARGET, CHECKINGATE}

    public pathingState pathState = pathingState.START;

    //State to return to after moveToPoint case
    public pathingState returnState;

    public double pathTimeOut = 30000;

    public void bluePathGen() {
        start_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(StartingPose, ShootingPose)
                )
                .setLinearHeadingInterpolation(StartingPose.getHeading(), ShootingPose.getHeading())
                .build();

        fire_to_spike1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(ShootingPose, BlueSpikeAInsidePose)
                )
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), BlueSpikeAInsidePose.getHeading())
                .build();

        fire_to_spike4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(ShootingPose, BlueSpikeDInsidePose)
                )
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), BlueSpikeDInsidePose.getHeading())
                .build();


        spike1_traversal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeAInsidePose, BlueSpikeAOutsidePose)
                )
                .setLinearHeadingInterpolation(BlueSpikeAInsidePose.getHeading(), BlueSpikeAOutsidePose.getHeading())
                .build();

        spike4_traversal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeDInsidePose, BlueSpikeDOutsidePose)
                )
                .setLinearHeadingInterpolation(BlueSpikeDInsidePose.getHeading(), BlueSpikeDOutsidePose.getHeading())
                .build();


        spike1_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeAOutsidePose, ShootingPose)
                )
                .setLinearHeadingInterpolation(BlueSpikeAOutsidePose.getHeading(), ShootingPose.getHeading() + pathOffset)
                .build();

        spike4_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeDOutsidePose, ShootingPose)
                )
                .setLinearHeadingInterpolation(BlueSpikeDOutsidePose.getHeading(), ShootingPose.getHeading() + pathOffset)
                .build();


        fire_to_park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(ShootingPose, ParkingPose)
                )
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), ParkingPose.getHeading())
                .build();
    }

    public void redPathGen() {
        start_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(StartingPose, ShootingPose)
                )
                .setLinearHeadingInterpolation(StartingPose.getHeading(), ShootingPose.getHeading())
                .build();

        fire_to_spike1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(ShootingPose, RedSpikeAInsidePose)
                )
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), RedSpikeAInsidePose.getHeading())
                .build();

        fire_to_spike4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(ShootingPose, RedSpikeDInsidePose)
                )
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), RedSpikeDInsidePose.getHeading())
                .build();


        spike1_traversal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(RedSpikeAInsidePose, RedSpikeAOutsidePose)
                )
                .setLinearHeadingInterpolation(RedSpikeAInsidePose.getHeading(), RedSpikeAOutsidePose.getHeading())
                .build();

        spike4_traversal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(RedSpikeDInsidePose, RedSpikeDOutsidePose)
                )
                .setLinearHeadingInterpolation(RedSpikeDInsidePose.getHeading(), RedSpikeDOutsidePose.getHeading())
                .build();


        spike1_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(RedSpikeAOutsidePose, ShootingPose)
                )
                .setLinearHeadingInterpolation(RedSpikeAOutsidePose.getHeading(), ShootingPose.getHeading() + pathOffset)
                .build();

        spike4_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeDOutsidePose, ShootingPose)
                )
                .setLinearHeadingInterpolation(BlueSpikeDOutsidePose.getHeading(), ShootingPose.getHeading() + pathOffset)
                .build();


        fire_to_park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(ShootingPose, ParkingPose)
                )
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), ParkingPose.getHeading())
                .build();
    }

    /**  Required OpMode Autonomous Control Methods  */
    public void stateVarableMainTelemetry(){
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Current time : ", opmodeTimer.getElapsedTime());
        telemetry.addData("Wait time : ", delayStartTime);

        telemetry.addData("Current state :", pathState);
        telemetry.addData("Spikes Intook", spikesTaken);
    }
    public void autoStateLoop() {

        //Max time breakout
        if (opmodeTimer.getElapsedTime() > maxTimeBreakout) {
            if (pathState != pathingState.END) {
                telemetry.addLine("EMERGENCY BREAKOUT. PARKING.");
                follower.followPath(fire_to_park);
                pathState = pathingState.PARK;
            }
        }

        switch (pathState) {
            case STARTDELAY:
                //Check if wait has been fulfilled
                if (delayStartTime <= opmodeTimer.getElapsedTime() - startDelay) {
                    pathState = pathingState.START;
                }
                break;


            case START:
                //Spin up motors
                decBot.intakeControl(intakeSpeed);
                decBot.flylaunch(targetVelocity);
                //Move to the first spike
                follower.followPath(start_to_fire);
                pathState = pathingState.PREFIRE;
                break;


            case PREFIRE:
                if (!follower.isBusy()) {
                    //Fire before any spike logic
                    moveToPointChain = null;
                    startFireTime = opmodeTimer.getElapsedTime();
                    returnState = pathingState.INTAKESPIKES;
                    pathState = pathingState.CHECKINGATE;
                }
                break;


            case INTAKESPIKES:
                //Intake Spike A
                if (spikesTaken == 0) {
                    pathState = pathingState.TAKESPIKEONE;
                    break;
                }

                //Intake Spike D
                if (spikesTaken == 1) {
                    //Take Spike D
                    pathState = pathingState.TAKESPIKEFOUR;
                }

                //Park
                if (spikesTaken == 2) {
                    //Go park
                    moveToPointChain = fire_to_park;
                    returnState = pathingState.PARK;
                    pathState = pathingState.MOVETOPOINT;
                    break;
                }
                break;

            case TAKESPIKEONE:
                //Start 1st spike intake
                if(moveToPointChain == null){
                    moveToPointChain = fire_to_spike1;
                    returnState = pathingState.TAKESPIKEONE;
                    pathState = pathingState.MOVETOPOINT;
                    break;
                }
                //1st spike inside to outside
                if(moveToPointChain == fire_to_spike1){
                    moveToPointChain = spike1_traversal;
                    returnState = pathingState.TAKESPIKEONE;
                    pathState = pathingState.INTAKETOPOINT;
                    break;
                }
                //1st spike to fire
                if(moveToPointChain == spike1_traversal){
                    moveToPointChain = spike1_to_fire;
                    returnState = pathingState.TAKESPIKEONE;
                    pathState = pathingState.MOVETOPOINT;
                    break;
                }
                //Fire ball
                if(moveToPointChain == spike1_to_fire){
                    moveToPointChain = null;
                    startFireTime = opmodeTimer.getElapsedTime();
                    returnState = pathingState.INTAKESPIKES;
                    spikesTaken += 1;
                    pathState = pathingState.CHECKINGATE;
                    break;
                }
                break;


            case TAKESPIKEFOUR:
                //Start 2nd spike intake
                if(moveToPointChain == null){
                    moveToPointChain = fire_to_spike4;
                    pathTimeOut = 4000;
                    returnState = pathingState.TAKESPIKEFOUR;
                    pathState = pathingState.MOVETOPOINT;
                    break;
                }
                //2nd spike inside to outside
                if(moveToPointChain == fire_to_spike4){
                    pathTimeOut = 3000;
                    moveToPointChain = spike4_traversal;
                    returnState = pathingState.TAKESPIKEFOUR;
                    pathState = pathingState.INTAKETOPOINT;
                    break;
                }
                //2nd spike to fire
                if(moveToPointChain == spike4_traversal){
                    moveToPointChain = spike4_to_fire;
                    returnState = pathingState.TAKESPIKEFOUR;
                    pathState = pathingState.MOVETOPOINT;
                    break;
                }
                //Fire ball
                if(moveToPointChain == spike4_to_fire){
                    moveToPointChain = null;
                    startFireTime = opmodeTimer.getElapsedTime();
                    returnState = pathingState.INTAKESPIKES;
                    spikesTaken += 1;
                    pathState = pathingState.CHECKINGATE;
                    break;
                }
                break;

            case MOVETOPOINT:
                //Uses moveToPointChain (Path) and returnState (pathingState)
                follower.followPath(moveToPointChain);
                telemetry.addData("Current Path:", moveToPointChain);
                pathState = pathingState.RETURNMOVETOPOINT;
                break;

            case INTAKETOPOINT:
                //Uses moveToPointChain (Path) and returnState (pathingState)
                follower.followPath(moveToPointChain, intakeMoveSpeed, true);
                telemetry.addData("Current Path:", moveToPointChain);

                pathState = pathingState.RETURNMOVETOPOINT;
                break;

            case RETURNMOVETOPOINT:
                //Just returns to the case
                if (!(follower.isBusy()) || pathTimeOut < opmodeTimer.getElapsedTime()) {
                    pathState = returnState;
                    pathTimeOut = 30000;
                }
                break;

            case CHECKINGATE:
                if(LaunchWheelsInGate(targetVelocity, variance)){
                    pathState = pathingState.FIREANDRETURNSTATE;
                }
                startFireTime = opmodeTimer.getElapsedTime();
                break;

            case FIREANDRETURNSTATE:
                //autotargeting
                limeLightData();
                autoTarget(xAutoOffset);
                if (burnerLaunch(opmodeTimer.getElapsedTime(), startFireTime, true)) {
                    pathState = returnState;
                }

                break;


            case PARK:
                decBot.intakeControl(0);
                decBot.flylaunch(0);
                if (!(follower.isBusy())) {
                    pathState = pathingState.END;
                }
                break;
            case END:
                break;
        }
    }
}


