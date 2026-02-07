package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.BlueAutoNew.Far;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.StateAutoMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.MainContraints;


@Autonomous(name = "Blue State Far 2 Spike", group = "Drive")
public class BlueStateFar2Spike extends StateAutoMain {

    //--------------Config for paths-----------------------
    //Start Pose
    public final Pose StartingPose = BlueFarStartPose;
    //Shoot Pose
    public final Pose ShootingPose = BlueFarShootPose;
    //Park Pose
    public final Pose ParkingPose = BlueFarParkPose;

    //Optional Pose for shooting after Third Spike
    public final Pose ThirdShootPose = BlueFarShootPose;

    //Delay before initial movement (ms)
    public final float startDelay = 0;

    //How many spikes are needed? 0-3
    public final int spikeAmount = 2;

    /*
    Order of intake
    true:
    C ----- ^
    B ----- |
    A ----- |
    false:
    C ----- |
    B ----- |
    A ----- V
    */
    public boolean AtoCIntake = true;

    //Base target velocity
    public double targetVelocity = 1520;

    //Variance in velocity permitted
    public double maxVariance = 5;

    //When to go to park as failsafe (0-30 seconds from start, recommended 25)
    public double maxTimeBreakout = 28 * 1000;
    //-----------------------------------------------------

    /**  Pedro Pathing Variables, Poses, Paths & States */
    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    //How many spikes have been intook
    public int spikesTaken = 0;

    //Time reference for burnerlaunch function
    public double startFireTime;

    //path for the moveToPoint case used in spike navigation
    public PathChain moveToPointChain;

    //Help view elapsed time on wait cases
    public float delayStartTime = 0;

    public Path scorePreload;

    protected PathChain start_to_fire;
    protected PathChain fire_to_spike1;
    protected PathChain fire_to_spike2;
    protected PathChain fire_to_spike3;
    protected PathChain spike1_traversal;
    protected PathChain spike2_traversal;
    protected PathChain spike3_traversal;
    protected PathChain spike1_to_fire;
    protected PathChain spike2_to_fire;
    protected PathChain spike3_to_fire;
    protected PathChain fire_to_park;
    protected PathChain third_spike_to_shoot;

    //Base intake spin speed
    public double intakeSpeed = 1;

    //Base motor power limit while intaking (0-1)
    public double intakeMoveSpeed = 0.3;

    //set up simple states
    public enum pathingState {STARTDELAY, START, INTAKESPIKES, FIRING, FIRINGDELAY, PARK, END, MOVETOPOINT, RETURNMOVETOPOINT, FIREANDRETURNSTATE, PREFIRE, INTAKETOPOINT, TAKESPIKEONE, TAKESPIKETWO, TAKESPIKETHREE}
    public pathingState pathState = pathingState.START;

    //State to return to after moveToPoint case
    public pathingState returnState;


    public void pathGen(){
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

        fire_to_spike2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(ShootingPose, BlueSpikeBInsidePose)
                )
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), BlueSpikeBInsidePose.getHeading())
                .build();

        fire_to_spike3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(ShootingPose, BlueSpikeCInsidePose)
                )
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), BlueSpikeCInsidePose.getHeading())
                .build();

        spike1_traversal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeAInsidePose, BlueSpikeAOutsidePose)
                )
                .setLinearHeadingInterpolation(BlueSpikeAInsidePose.getHeading(), BlueSpikeAOutsidePose.getHeading())
                .build();

        spike2_traversal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeBInsidePose, BlueSpikeBOutsidePose)
                )
                .setLinearHeadingInterpolation(BlueSpikeBInsidePose.getHeading(), BlueSpikeBOutsidePose.getHeading())
                .build();

        spike3_traversal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeCInsidePose, BlueSpikeCOutsidePose)
                )
                .setLinearHeadingInterpolation(BlueSpikeCInsidePose.getHeading(), BlueSpikeCOutsidePose.getHeading())
                .build();

        spike1_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeAOutsidePose, BlueFarShootPose)
                )
                .setLinearHeadingInterpolation(BlueSpikeAOutsidePose.getHeading(), BlueFarShootPose.getHeading())
                .build();

        spike2_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeBOutsidePose, ShootingPose)
                )
                .setLinearHeadingInterpolation(BlueSpikeBOutsidePose.getHeading(), ShootingPose.getHeading())
                .build();

        spike3_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeCOutsidePose, ShootingPose)
                )
                .setLinearHeadingInterpolation(BlueSpikeCOutsidePose.getHeading(), ShootingPose.getHeading())
                .build();

        fire_to_park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(ShootingPose, ParkingPose)
                )
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), ParkingPose.getHeading())
                .build();

        if(AtoCIntake){
            third_spike_to_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(BlueSpikeCOutsidePose, ThirdShootPose)
                    )
                    .setLinearHeadingInterpolation(BlueSpikeCOutsidePose.getHeading(), ThirdShootPose.getHeading())
                    .build();
        }
        else{
            third_spike_to_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(BlueSpikeAOutsidePose, ThirdShootPose)
                    )
                    .setLinearHeadingInterpolation(BlueSpikeAOutsidePose.getHeading(), ThirdShootPose.getHeading())
                    .build();
        }
    }

    /**  Required OpMode Autonomous Control Methods  */

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        decBot.initRobot(hardwareMap);
        follower = MainContraints.createFollower(hardwareMap);
        pathGen();
        follower.setStartingPose(StartingPose);
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        pathState = pathingState.STARTDELAY;
        //Get the time of the starting delay
        delayStartTime = opmodeTimer.getElapsedTime();
        spikesTaken = 0;
    }



    @Override
    public void loop() {
        follower.update();
        AutoMainTelemetry();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Current time : ", opmodeTimer.getElapsedTime());
        telemetry.addData("Wait time : ", delayStartTime);
        telemetry.addData("Current state :", pathState);
        telemetry.addData("Spikes Intook", spikesTaken);
        telemetry.update();

        //Max time breakout
        if(opmodeTimer.getElapsedTime() > maxTimeBreakout){
            if(pathState != pathingState.END && pathState != pathingState.RETURNMOVETOPOINT && pathState != pathingState.MOVETOPOINT){
                telemetry.addLine("EMERGENCY BREAKOUT. PARKING.");
                //Go park
                moveToPointChain = fire_to_park;
                returnState = pathingState.PARK;
                pathState = pathingState.MOVETOPOINT;
            }
        }


        // very simple movment test
        switch (pathState) {
            case STARTDELAY:
                //Check if wait has been fulfilled
                if(delayStartTime <= opmodeTimer.getElapsedTime() - startDelay) {
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
                if(!follower.isBusy()) {
                    //Fire before any spike logic
                    moveToPointChain = null;
                    startFireTime = opmodeTimer.getElapsedTime();
                    returnState = pathingState.INTAKESPIKES;
                    pathState = pathingState.FIREANDRETURNSTATE;
                }
                break;


            case INTAKESPIKES:
                //If you need to intake at all
                if(spikesTaken < spikeAmount)
                {
                    //Intake Spike A or C depending on bool AtoCIntake
                    if(spikesTaken == 0){
                        pathState = AtoCIntake ? pathingState.TAKESPIKEONE : pathingState.TAKESPIKETHREE;
                        break;
                    }

                    //Intake Spike B
                    if(spikesTaken == 1) {
                        //Take Spike B
                        pathState = pathingState.TAKESPIKETWO;
                    }
                    //Intake Spike C or A depending on bool AtoCIntake
                    if(spikesTaken == 2){
                        pathState = AtoCIntake ? pathingState.TAKESPIKETHREE : pathingState.TAKESPIKEONE;
                        break;
                    }
                    break;
                }
                else{
                    //Go park
                    moveToPointChain = fire_to_park;
                    returnState = pathingState.PARK;
                    pathState = pathingState.MOVETOPOINT;
                    break;
                }

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
                    if(spikesTaken == 2){
                        moveToPointChain = third_spike_to_shoot;
                    }
                    else {
                        moveToPointChain = spike1_to_fire;
                    }
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
                    pathState = pathingState.FIREANDRETURNSTATE;
                    break;
                }
                if(moveToPointChain == third_spike_to_shoot){
                    moveToPointChain = null;
                    startFireTime = opmodeTimer.getElapsedTime();
                    returnState = pathingState.INTAKESPIKES;
                    spikesTaken += 1;
                    pathState = pathingState.FIREANDRETURNSTATE;
                    break;
                }
                break;


            case TAKESPIKETWO:
                //Start 2nd spike intake
                if(moveToPointChain == null){
                    moveToPointChain = fire_to_spike2;
                    returnState = pathingState.TAKESPIKETWO;
                    pathState = pathingState.MOVETOPOINT;
                    break;
                }
                //2nd spike inside to outside
                if(moveToPointChain == fire_to_spike2){
                    moveToPointChain = spike2_traversal;
                    returnState = pathingState.TAKESPIKETWO;
                    pathState = pathingState.INTAKETOPOINT;
                    break;
                }
                //2nd spike to fire
                if(moveToPointChain == spike2_traversal){
                    moveToPointChain = spike2_to_fire;
                    returnState = pathingState.TAKESPIKETWO;
                    pathState = pathingState.MOVETOPOINT;
                    break;
                }
                //Fire ball
                if(moveToPointChain == spike2_to_fire){
                    moveToPointChain = null;
                    startFireTime = opmodeTimer.getElapsedTime();
                    returnState = pathingState.INTAKESPIKES;
                    spikesTaken += 1;
                    pathState = pathingState.FIREANDRETURNSTATE;
                    break;
                }
                break;

            case TAKESPIKETHREE:
                //Start 3rd spike intake
                if(moveToPointChain == null){
                    moveToPointChain = fire_to_spike3;
                    returnState = pathingState.TAKESPIKETHREE;
                    pathState = pathingState.MOVETOPOINT;
                    break;
                }
                //3rd spike inside to outside
                if(moveToPointChain == fire_to_spike3){
                    moveToPointChain = spike3_traversal;
                    returnState = pathingState.TAKESPIKETHREE;
                    pathState = pathingState.INTAKETOPOINT;
                    break;
                }
                //3rd spike to fire
                if(moveToPointChain == spike3_traversal){
                    if(spikesTaken == 2){
                        moveToPointChain = third_spike_to_shoot;
                    }
                    else {
                        moveToPointChain = spike3_to_fire;
                    }
                    returnState = pathingState.TAKESPIKETHREE;
                    pathState = pathingState.MOVETOPOINT;
                    break;
                }
                //Fire ball
                if(moveToPointChain == spike3_to_fire){
                    moveToPointChain = null;
                    startFireTime = opmodeTimer.getElapsedTime();
                    returnState = pathingState.INTAKESPIKES;
                    spikesTaken += 1;
                    pathState = pathingState.FIREANDRETURNSTATE;
                    break;
                }
                if(moveToPointChain == third_spike_to_shoot){
                    moveToPointChain = null;
                    startFireTime = opmodeTimer.getElapsedTime();
                    returnState = pathingState.INTAKESPIKES;
                    spikesTaken += 1;
                    pathState = pathingState.FIREANDRETURNSTATE;
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
                if (!(follower.isBusy())) {
                    pathState = returnState;
                }
                break;

            case FIREANDRETURNSTATE:
                if(burnerLaunch(opmodeTimer.getElapsedTime(), startFireTime, true))
                {
                    pathState = returnState;
                }
                break;


            case PARK:
                //Move to the first spike
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

