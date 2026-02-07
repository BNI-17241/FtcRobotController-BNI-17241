package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.Old.RedAlliance;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.StateAutoMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.ProgramConstants;

@Disabled
@Autonomous(name = "9 ball auto hard code red", group = "Drive")
public class NineBallStateAutoHardCodeRed extends StateAutoMain {

    public Follower follower;
    public Timer opmodeTimer;

    protected Limelight3A limelight;



    // init for april tag value so its public in case needed in outher systems
    public int april_tag_value;

    protected PathChain start_to_fire_location;
    protected PathChain fire_location_to_inside_Spike_A;
    protected PathChain ball_inside_to_ball_outside_Spike_A;
    protected PathChain ball_outside_A_to_fire_location;
    protected PathChain fire_location_to_inside_SPike_B;
    protected PathChain ball_inside_to_ball_outside_Spike_B;
    protected PathChain ball_outside_B_to_fire_location;
    protected PathChain firing_location_to_park;



    protected double maxTime = 25.0;

    public boolean MaxTimeBreakout(){
        if (opmodeTimer.getElapsedTime() >= maxTime){
            return true;
        }
        return false;
    }

    public void pathGen() {


        start_to_fire_location = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(RedFarStartPose, RedFarShootPose)
                )
                .setLinearHeadingInterpolation(RedFarStartPose.getHeading(), RedFarShootPose.getHeading())
                .build();
        fire_location_to_inside_Spike_A = follower.
                pathBuilder().
                addPath(new BezierLine(RedFarShootPose, RedSpikeAInsidePose))
                .setLinearHeadingInterpolation(RedFarShootPose.getHeading(), RedSpikeAInsidePose.getHeading())
                .build();

        ball_inside_to_ball_outside_Spike_A = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(RedSpikeAInsidePose, RedSpikeAOutsidePose)
                )
                .setConstantHeadingInterpolation(RedSpikeBInsidePose.getHeading())
                .build();

        ball_outside_A_to_fire_location = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(RedSpikeAOutsidePose, RedFarShootPose)
                )
                .setLinearHeadingInterpolation(RedSpikeAOutsidePose.getHeading(), RedFarShootPose.getHeading())
                .build();
        fire_location_to_inside_SPike_B = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(RedFarShootPose, RedSpikeBInsidePose)
                )
                .setLinearHeadingInterpolation(RedFarShootPose.getHeading(), RedSpikeBInsidePose.getHeading())
                .build();
        ball_inside_to_ball_outside_Spike_B = follower.
                pathBuilder()
                .addPath(
                        new BezierLine(RedSpikeBInsidePose, RedSpikeBOutsidePose)
                )
                .setConstantHeadingInterpolation(RedSpikeAInsidePose.getHeading())
                .build();
        ball_outside_B_to_fire_location = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(RedSpikeBOutsidePose, RedFarShootPose)
                )
                .setLinearHeadingInterpolation(RedSpikeBOutsidePose.getHeading(), RedFarShootPose.getHeading())
                .build();
        firing_location_to_park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(RedFarShootPose, RedFarParkPose)
                )
                .setLinearHeadingInterpolation(RedFarShootPose.getHeading(), RedFarParkPose.getHeading())
                .build();

    }
    private void telemetry(){
        AutoMainTelemetry();
        telemetry.addData("pathState", pathState);
        telemetry.addData("april tag", april_tag_value);
        telemetry.update();
    }

    @Override
    public void init() {
        decBot.initRobot(hardwareMap);
        opmodeTimer = new Timer();
        follower = ProgramConstants.createFollower(hardwareMap);
        follower.setStartingPose(RedFarStartPose);
        pathGen();
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
    }

    //set up simple states
    public enum pathingState {START, First_Firing ,INSIDE_A, OUTSIDE_A, SECOND_FIRING, PARK, FINISHED, INSIDE_B, OUTSIDE_B, Third_Fire}
    public pathingState pathState = pathingState.START;

    @Override
    public void loop() {
        follower.update();
        telemetry();

        // very simple movment test
        switch (pathState) {
            case START:
                follower.followPath(start_to_fire_location);
                pathState = pathingState.First_Firing;
                break;
            case First_Firing:
                if (!(follower.isBusy())) {
                    if (LaunchBalls(900)) {
                        follower.followPath(fire_location_to_inside_Spike_A);
                        pathState = pathingState.INSIDE_A;
                        decBot.stopFeed();
                    }
                }
                break;
            case INSIDE_A:
                if (!(follower.isBusy())) {
                    follower.followPath(ball_inside_to_ball_outside_Spike_A);
                    pathState = pathingState.OUTSIDE_A;
                }
                break;
            case OUTSIDE_A:
                if (!(follower.isBusy())) {
                    follower.followPath(ball_outside_A_to_fire_location);
                    pathState = pathingState.SECOND_FIRING;
                    decBot.intakeControl(0);
                }
                break;
            case SECOND_FIRING:
                if (MaxTimeBreakout()){
                    pathState = pathingState.PARK;
                    decBot.flylaunch(0);
                    decBot.stopFeed();
                }
                if (!(follower.isBusy())) {
                    if (LaunchBalls(900)) {
                        follower.followPath(fire_location_to_inside_SPike_B);
                        decBot.flylaunch(0);
                        pathState = pathingState.INSIDE_B;
                    }
                }
                break;
            case INSIDE_B:
                if (!(follower.isBusy())) {
                    follower.followPath(ball_inside_to_ball_outside_Spike_B);
                    pathState = pathingState.OUTSIDE_B;
                }
                break;
            case OUTSIDE_B:
                if (!(follower.isBusy())) {
                    follower.followPath(ball_outside_B_to_fire_location);
                    pathState = pathingState.Third_Fire;
                    decBot.intakeControl(0);
                }
                break;
            case Third_Fire:
                if (MaxTimeBreakout()){
                    pathState = pathingState.PARK;
                    decBot.flylaunch(0);
                    decBot.stopFeed();
                }
                if (!(follower.isBusy())) {
                    if (LaunchBalls(900)) {
                        follower.followPath(fire_location_to_inside_SPike_B);
                        decBot.flylaunch(0);
                        pathState = pathingState.PARK;
                    }
                }
                break;
            case PARK:
                if (!(follower.isBusy())) {
                    follower.followPath(firing_location_to_park);
                    pathState = pathingState.FINISHED;
                }
                break;
            case FINISHED:
                break;
        }
    }
}

//I added coment
