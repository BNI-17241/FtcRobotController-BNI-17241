package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.BlueAutoNew;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoMainNew;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.ProgramConstants;

@Autonomous(name = "9 ball auto hard code blue", group = "Drive")
public class NineBallAutoHardCodeBlue extends AutoMainNew {

    public Follower follower;
    protected Limelight3A limelight;

    public int april_tag_value;

    protected PathChain start_to_fire_location;
    protected PathChain fire_location_to_inside_Spike_A;
    protected PathChain ball_inside_to_ball_outside_Spike_A;
    protected PathChain ball_outside_A_to_fire_location;

    protected PathChain fire_location_to_inside_SPike_B;
    protected PathChain ball_inside_to_ball_outside_Spike_B;
    protected PathChain ball_outside_B_to_fire_location;

    protected PathChain firing_location_to_park;

    public void pathGen() {

        start_to_fire_location = follower.pathBuilder()
                .addPath(new BezierLine(BlueFarStartPose, BlueFarShootPose))
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueFarShootPose.getHeading())
                .build();

        fire_location_to_inside_Spike_A = follower.pathBuilder()
                .addPath(new BezierLine(BlueFarShootPose, BlueSpikeAInsidePose))
                .setLinearHeadingInterpolation(BlueFarShootPose.getHeading(), BlueSpikeAInsidePose.getHeading())
                .build();

        ball_inside_to_ball_outside_Spike_A = follower.pathBuilder()
                .addPath(new BezierLine(BlueSpikeAInsidePose, BlueSpikeAOutsidePose))
                .setConstantHeadingInterpolation(BlueSpikeAInsidePose.getHeading()) // FIX
                .build();

        ball_outside_A_to_fire_location = follower.pathBuilder()
                .addPath(new BezierLine(BlueSpikeAOutsidePose, BlueFarShootPose))
                .setLinearHeadingInterpolation(BlueSpikeAOutsidePose.getHeading(), BlueFarShootPose.getHeading())
                .build();

        fire_location_to_inside_SPike_B = follower.pathBuilder()
                .addPath(new BezierLine(BlueFarShootPose, BlueSpikeBInsidePose))
                .setLinearHeadingInterpolation(BlueFarShootPose.getHeading(), BlueSpikeBInsidePose.getHeading())
                .build();

        ball_inside_to_ball_outside_Spike_B = follower.pathBuilder()
                .addPath(new BezierLine(BlueSpikeBInsidePose, BlueSpikeBOutsidePose))
                .setConstantHeadingInterpolation(BlueSpikeBInsidePose.getHeading()) // FIX
                .build();

        ball_outside_B_to_fire_location = follower.pathBuilder()
                .addPath(new BezierLine(BlueSpikeBOutsidePose, BlueFarShootPose))
                .setLinearHeadingInterpolation(BlueSpikeBOutsidePose.getHeading(), BlueFarShootPose.getHeading())
                .build();

        firing_location_to_park = follower.pathBuilder()
                .addPath(new BezierLine(BlueFarShootPose, BlueFarParkPose))
                .setLinearHeadingInterpolation(BlueFarShootPose.getHeading(), BlueFarParkPose.getHeading())
                .build();
    }

    private void telemetryCustom() {
        AutoMainTelemetry();
        telemetry.addData("pathState", pathState);
        telemetry.addData("april tag", april_tag_value);
        telemetry.update();
    }

    @Override
    public void init() {
        decBot.initRobot(hardwareMap);
        follower = ProgramConstants.createFollower(hardwareMap);
        follower.setStartingPose(BlueFarStartPose);
        pathGen();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
    }

    public enum pathingState {
        START,
        First_Firing,
        INSIDE_A,
        OUTSIDE_A,
        SECOND_FIRING,
        INSIDE_B,
        OUTSIDE_B,
        Third_Fire,
        PARK,
        FINISHED
    }

    public pathingState pathState = pathingState.START;

    @Override
    public void loop() {
        follower.update();
        telemetryCustom();

        switch (pathState) {
            case START:
                follower.followPath(start_to_fire_location);
                pathState = pathingState.First_Firing;
                break;

            case First_Firing:
                if (!follower.isBusy()) {
                    if (LaunchBalls(900)) {
                        follower.followPath(fire_location_to_inside_Spike_A);
                        decBot.stopFeed();
                        pathState = pathingState.INSIDE_A;
                    }
                }
                break;

            case INSIDE_A:
                if (!follower.isBusy()) {
                    follower.followPath(ball_inside_to_ball_outside_Spike_A);
                    pathState = pathingState.OUTSIDE_A;
                }
                break;

            case OUTSIDE_A:
                if (!follower.isBusy()) {
                    follower.followPath(ball_outside_A_to_fire_location);
                    decBot.intakeControl(0);
                    pathState = pathingState.SECOND_FIRING;
                }
                break;

            case SECOND_FIRING:
                if (!follower.isBusy()) {
                    if (LaunchBalls(900)) {
                        follower.followPath(fire_location_to_inside_SPike_B);
                        decBot.flylaunch(0);
                        pathState = pathingState.INSIDE_B;
                    }
                }
                break;

            case INSIDE_B:
                if (!follower.isBusy()) {
                    follower.followPath(ball_inside_to_ball_outside_Spike_B);
                    pathState = pathingState.OUTSIDE_B;
                }
                break;

            case OUTSIDE_B:
                if (!follower.isBusy()) {
                    follower.followPath(ball_outside_B_to_fire_location);
                    decBot.intakeControl(0);
                    pathState = pathingState.Third_Fire;
                }
                break;

            case Third_Fire:
                if (!follower.isBusy()) {
                    if (LaunchBalls(900)) {
                        // FIX: go park after third fire
                        follower.followPath(firing_location_to_park);
                        decBot.flylaunch(0);
                        pathState = pathingState.PARK;
                    }
                }
                break;

            case PARK:
                if (!follower.isBusy()) {
                    pathState = pathingState.FINISHED;
                }
                break;

            case FINISHED:
                // optionally stop everything here
                break;
        }
    }
}
