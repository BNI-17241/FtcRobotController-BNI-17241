package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.BlueAutoNew;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.StateAutoMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.MainContraints;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.ProgramConstants;


@Autonomous(name = "9 ball auto hard code blue", group = "Drive")
public class NineBallStateAutoHardCodeBlue extends StateAutoMain {

    public Follower follower;

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
    protected double startFireTime;
//    public boolean MaxTimeBreakout(){
//        if (opmodeTimer.getElapsedTime() >= maxTime){
//            return true;
//        }
//        return false;
//    }

    public void pathGen() {


        start_to_fire_location = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarStartPose, BlueFarShootPose)
                )
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueFarShootPose.getHeading())
                .build();
        fire_location_to_inside_Spike_A = follower.
                pathBuilder().
                addPath(new BezierLine(BlueFarShootPose, BlueSpikeAInsidePose))
                .setLinearHeadingInterpolation(BlueFarShootPose.getHeading(), BlueSpikeAInsidePose.getHeading())
                .build();

        ball_inside_to_ball_outside_Spike_A = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeAInsidePose, BlueSpikeAOutsidePose)
                )
                .setConstantHeadingInterpolation(BlueSpikeBInsidePose.getHeading())
                .build();

        ball_outside_A_to_fire_location = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeAOutsidePose, BlueFarShootPose)
                )
                .setLinearHeadingInterpolation(BlueSpikeAOutsidePose.getHeading(), BlueFarShootPose.getHeading())
                .build();
        fire_location_to_inside_SPike_B = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarShootPose, BlueSpikeBInsidePose)
                )
                .setLinearHeadingInterpolation(BlueFarShootPose.getHeading(), BlueSpikeBInsidePose.getHeading())
                .build();
        ball_inside_to_ball_outside_Spike_B = follower.
                pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeBInsidePose, BlueSpikeBOutsidePose)
                )
                .setConstantHeadingInterpolation(BlueSpikeAInsidePose.getHeading())
                .build();
        ball_outside_B_to_fire_location = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueSpikeBOutsidePose, BlueFarShootPose)
                )
                .setLinearHeadingInterpolation(BlueSpikeBOutsidePose.getHeading(), BlueFarShootPose.getHeading())
                .build();
        firing_location_to_park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarShootPose, BlueFarParkPose)
                )
                .setLinearHeadingInterpolation(BlueFarShootPose.getHeading(), BlueFarParkPose.getHeading())
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
        follower.setStartingPose(BlueFarStartPose);
        pathGen();
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
    }

    //set up simple states
    public enum pathingState {START, First_Firing ,INSIDE_A, OUTSIDE_A, SECOND_FIRING, PARK, FINISHED, INSIDE_B, OUTSIDE_B, Third_Fire}
    public pathingState pathState = pathingState.START;
    protected boolean hasStarted = false;
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
                    if (!hasStarted){
                        startFireTime = opmodeTimer.getElapsedTime();
                        hasStarted = true;
                    }
                    if (burnerLaunch(opmodeTimer.getElapsedTime(), startFireTime)) {
                        hasStarted = false;
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
//                if (MaxTimeBreakout()){
//                    pathState = pathingState.PARK;
//                    decBot.flylaunch(0);
//                    decBot.stopFeed();
//                }
                if (!(follower.isBusy())) {
                    if (!hasStarted){
                        startFireTime = opmodeTimer.getElapsedTime();
                        hasStarted = true;
                    }
                    if (burnerLaunch(opmodeTimer.getElapsedTime(), startFireTime)) {
                        hasStarted = false;
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
//                if (MaxTimeBreakout()){
//                    pathState = pathingState.PARK;
//                    decBot.flylaunch(0);
//                    decBot.stopFeed();
//                }
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
