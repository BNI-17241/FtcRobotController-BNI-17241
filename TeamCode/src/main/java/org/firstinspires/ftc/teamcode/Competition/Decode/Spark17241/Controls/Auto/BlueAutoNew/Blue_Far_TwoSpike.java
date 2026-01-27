package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.BlueAutoNew;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoMainNew;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.MainContraints;


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
    protected PathChain spike1_traversal;
    protected PathChain spike1_to_fire;
    protected PathChain fire_to_spike2;
    protected PathChain spike2_traversal;
    protected PathChain spike2_to_fire;
    protected PathChain fire_to_park;




    //set up simple states
    public enum pathingState {STARTDELAY, START, INTAKESPIKES, FIRING, FIRINGDELAY, PARK, END, MOVETOPOINT, RETURNMOVETOPOINT, FIREANDRETURNSTATE}
    public pathingState pathState = pathingState.START;

    //State to return to after moveToPoint case
    public pathingState returnState;


    public void pathGen(){
        start_to_spike1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarStartPose, BlueSpikeAInsidePose)
                )
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueMidShootPose.getHeading())
                .build();

        spike1_traversal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeAInsidePose, BlueSpikeAOutsidePose)
                )
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueMidShootPose.getHeading())
                .build();

        spike1_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeAOutsidePose, BlueFarShootPose)
                )
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueMidShootPose.getHeading())
                .build();

        fire_to_spike2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarShootPose, BlueSpikeBInsidePose)
                )
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueMidShootPose.getHeading())
                .build();

        spike2_traversal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeBInsidePose, BlueSpikeBOutsidePose)
                )
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueMidShootPose.getHeading())
                .build();

        spike2_to_fire = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeBOutsidePose, BlueFarShootPose)
                )
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueMidShootPose.getHeading())
                .build();

        fire_to_park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarShootPose, BlueFarParkPose)
                )
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueMidShootPose.getHeading())
                .build();
    }

    /**  Required OpMode Autonomous Control Methods  */

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = MainContraints.createFollower(hardwareMap);
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
                //Move to the first spike
                follower.followPath(start_to_spike1);
                pathState = pathingState.INTAKESPIKES;
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
                                moveToPointChain = start_to_spike1;
                                returnState = pathingState.INTAKESPIKES;
                                pathState = pathingState.MOVETOPOINT;
                                break;
                            }
                            //1st spike inside to outside
                            if(moveToPointChain == start_to_spike1){
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
                                returnState = pathingState.INTAKESPIKES;
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
                                returnState = pathingState.END;
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
                            returnState = pathingState.INTAKESPIKES;
                            pathState = pathingState.FIREANDRETURNSTATE;
                            break;
                        }
                    }
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
                if(LaunchBalls())
                {
                    pathState = returnState;
                    spikesTaken += 1;
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
                if (!(follower.isBusy())) {
                    pathState = pathingState.END;
                }
                break;
            case END:
                break;
        }
    }
}

