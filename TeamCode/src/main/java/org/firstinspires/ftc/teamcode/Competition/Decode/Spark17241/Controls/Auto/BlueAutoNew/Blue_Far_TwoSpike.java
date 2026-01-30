package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.BlueAutoNew;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoMainNew;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.ProgramConstants;


@Autonomous(name = "Blue: Far Two Spike", group = "Drive")
public class Blue_Far_TwoSpike extends AutoMainNew {
//
    /**  Pedro Pathing Variables, Poses, Paths & States */
    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    //Delay before intial movement (ms)
    public final float startDelay = 3000;
    //Delay at goal after firing (ms)
    public final float fireDelay  = 5000;

    //How many spikes are needed? 0-2
    public final int spikeAmount = 2;

    //Initial fire check
    public boolean initialFire = false;

    //How many spikes have been intook
    public int spikesTaken = 0;

    public double startFireTime;

    public boolean AtoCIntake = true;


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

    //path for the moveToPoint case used in spike navigation
    public PathChain moveToPointChain;

    //Help view elapsed time on wait cases
    public float delayStartTime = 0;

    public Path scorePreload;

    protected PathChain start_to_spike1;
    protected PathChain start_to_fire;
    protected PathChain fire_to_spike1;
    protected PathChain spike1_traversal;
    protected PathChain spike1_to_fire;
    protected PathChain fire_to_spike2;
    protected PathChain spike2_traversal;
    protected PathChain spike2_to_fire;
    protected PathChain fire_to_park;


    //Base target velocity
    public double targetVelocity = 900;

    //Base intake spin speed
    public double intakeSpeed = 1;

    //set up simple states
    public enum pathingState {STARTDELAY, START, INTAKESPIKES, FIRING, FIRINGDELAY, PARK, END, MOVETOPOINT, RETURNMOVETOPOINT, FIREANDRETURNSTATE, PREFIRE}
    public pathingState pathState = pathingState.START;

    //State to return to after moveToPoint case
    public pathingState returnState;


    public void pathGen(){
        start_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarStartPose, BlueFarShootPose)
                )
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueFarShootPose.getHeading())
                .build();

        fire_to_spike1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarShootPose, BlueSpikeAInsidePose)
                )
                .setLinearHeadingInterpolation(BlueFarShootPose.getHeading(), BlueSpikeAInsidePose.getHeading())
                .build();

        start_to_spike1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarStartPose, BlueSpikeAInsidePose)
                )
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueSpikeAInsidePose.getHeading())
                .build();

        spike1_traversal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeAInsidePose, BlueSpikeAOutsidePose)
                )
                .setLinearHeadingInterpolation(BlueSpikeAInsidePose.getHeading(), BlueSpikeAOutsidePose.getHeading())
                .build();

        spike1_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeAOutsidePose, BlueFarShootPose)
                )
                .setLinearHeadingInterpolation(BlueSpikeAOutsidePose.getHeading(), BlueFarShootPose.getHeading())
                .build();

        fire_to_spike2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarShootPose, BlueSpikeBInsidePose)
                )
                .setLinearHeadingInterpolation(BlueFarShootPose.getHeading(), BlueSpikeBInsidePose.getHeading())
                .build();

        spike2_traversal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeBInsidePose, BlueSpikeBOutsidePose)
                )
                .setLinearHeadingInterpolation(BlueSpikeBInsidePose.getHeading(), BlueSpikeBOutsidePose.getHeading())
                .build();

        spike2_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeBOutsidePose, BlueFarShootPose)
                )
                .setLinearHeadingInterpolation(BlueSpikeBOutsidePose.getHeading(), BlueFarShootPose.getHeading())
                .build();

        fire_to_park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarShootPose, BlueFarParkPose)
                )
                .setLinearHeadingInterpolation(BlueFarShootPose.getHeading(), BlueFarParkPose.getHeading())
                .build();
    }

    /**  Required OpMode Autonomous Control Methods  */

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        decBot.initRobot(hardwareMap);
        follower = ProgramConstants.createFollower(hardwareMap);
        pathGen();
        follower.setStartingPose(BlueFarStartPose);
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
        //telemetry.addData("Pose", Pose)

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
                //Fire before any spike logic
                moveToPointChain = null;
                startFireTime = opmodeTimer.getElapsedTime();
                returnState = pathingState.INTAKESPIKES;
                pathState = pathingState.FIREANDRETURNSTATE;
                break;


            case INTAKESPIKES:
                //If you need to intake at all
                if(spikesTaken < spikeAmount)
                {
                    //Intake Spike A or C depending on bool AtoCIntake
                    if(spikesTaken == 0){
                        if(AtoCIntake){
                            //Start 1st spike intake
                            if(moveToPointChain == null){
                                moveToPointChain = fire_to_spike1;
                                returnState = pathingState.INTAKESPIKES;
                                pathState = pathingState.MOVETOPOINT;
                                break;
                            }
                            //1st spike inside to outside
                            if(moveToPointChain == fire_to_spike1){
                                moveToPointChain = spike1_traversal;
                                returnState = pathingState.INTAKESPIKES;
                                pathState = pathingState.MOVETOPOINT;
                                break;
                            }
                            //1st spike to fire
                            if(moveToPointChain == spike1_traversal){
                                moveToPointChain = spike1_to_fire;
                                returnState = pathingState.INTAKESPIKES;
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
                        }
                        else{
                            //Start 3rd spike intake
                            if(moveToPointChain == null){
                                moveToPointChain = start_to_spike1;
                                returnState = pathingState.INTAKESPIKES;
                                pathState = pathingState.MOVETOPOINT;
                                break;
                            }
                            //3rd spike inside to outside
                            if(moveToPointChain == start_to_spike1){
                                moveToPointChain = spike1_traversal;
                                returnState = pathingState.INTAKESPIKES;
                                pathState = pathingState.MOVETOPOINT;
                                break;
                            }
                            //3rd spike to fire
                            if(moveToPointChain == spike1_traversal){
                                moveToPointChain = spike1_to_fire;
                                returnState = pathingState.INTAKESPIKES;
                                pathState = pathingState.MOVETOPOINT;
                                break;
                            }
                        }
                    }
                    //Intake Spike B
                    if(spikesTaken == 1){
                        //Start 2nd spike intake
                        if(moveToPointChain == null){
                            moveToPointChain = fire_to_spike2;
                            returnState = pathingState.INTAKESPIKES;
                            pathState = pathingState.MOVETOPOINT;
                            break;
                        }
                        //2nd spike inside to outside
                        if(moveToPointChain == fire_to_spike2){
                            moveToPointChain = spike2_traversal;
                            returnState = pathingState.INTAKESPIKES;
                            pathState = pathingState.MOVETOPOINT;
                            break;
                        }
                        //2nd spike to fire
                        if(moveToPointChain == spike2_traversal){
                            moveToPointChain = spike2_to_fire;
                            returnState = pathingState.INTAKESPIKES;
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
                    }
                }
                else{
                    moveToPointChain = fire_to_park;
                    returnState = pathingState.PARK;
                    pathState = pathingState.MOVETOPOINT;
                    break;
                }
                break;


            case MOVETOPOINT:
                //Uses moveToPointChain (Path) and returnState (pathingState)
                follower.followPath(moveToPointChain);
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

                if(burnerLaunch(targetVelocity, opmodeTimer.getElapsedTime(), startFireTime))
                {
                    pathState = returnState;

                }

                break;

            case FIRING:
                if (!(follower.isBusy())) {
                    pathState = pathingState.FIRINGDELAY;
                    //Reset delay
                    delayStartTime = opmodeTimer.getElapsedTime();
                }
                break;

            case FIRINGDELAY:
                //Check if wait has been fulfilled
                if(delayStartTime <= opmodeTimer.getElapsedTime() - fireDelay) {
                    //follower.followPath();
                    pathState = pathingState.PARK;
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

