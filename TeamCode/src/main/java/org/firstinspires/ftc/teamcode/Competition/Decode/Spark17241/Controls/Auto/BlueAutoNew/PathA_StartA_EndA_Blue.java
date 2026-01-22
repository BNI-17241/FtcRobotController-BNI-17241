package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.BlueAutoNew;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoMainNew;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;

import java.util.List;


@Autonomous(name = "PathA_StartA_EndA_Blue", group = "Drive")
public class PathA_StartA_EndA_Blue extends AutoMainNew {

    public Follower follower;
    public Timer pathTimer, opmodeTimer;


    protected Limelight3A limelight;

    public Pose Inside_pose;
    public Pose Outside_pose;
    // SIMPLE POSES
    public final Pose StartPose = new Pose(42, 8.5, Math.toRadians(90));
    public final Pose FiringPose = new Pose(58, 90, Math.toRadians(135));
    public final Pose ParkPose = new Pose(45, 18, Math.toRadians(90));

    public final Pose ControlPointFiringLocation = new Pose(94, 30);

    // init for april tag value so its public in case needed in outher systems
    public int april_tag_value;


//    protected final Pose PPGPose = new Pose(48, 81, Math.toRadians(180)); // closest to human 21
//    protected final Pose PPGPosePickup = new Pose(32, 81, Math.toRadians(180));
//
//    protected final Pose PGPPose = new Pose(48, 57.5, Math.toRadians(180)); // secount clostest 22
//    protected final Pose PGPPosePickup = new Pose(32, 57.5, Math.toRadians(180));
//
//    protected final Pose GPPPose = new Pose(48, 34, Math.toRadians(180)); // third closest 23
//    protected final Pose GPPPosePickup = new Pose(32, 34, Math.toRadians(180));

    protected PathChain start_to_fire_location;
    protected PathChain fire_location_to_ball_inside;
    protected PathChain ball_inside_to_ball_outside;
    protected PathChain ball_outside_to_fire_location;
    protected PathChain fire_location_to_park;

    public int limeLightData() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if(fr.getFiducialId() == 21 || fr.getFiducialId() == 22 || fr.getFiducialId() == 23){
                    return fr.getFiducialId();
                }
            }
        }
        return 21;
    }

    public void Paths_generation(int april_tag) {
        // definie inside/ outside (aka where

        if (april_tag == 23){
            Inside_pose = new Pose(97, 36, Math.toRadians(0)); // closest to human 21
            Outside_pose = new Pose(128, 36, Math.toRadians(0));
        }
        if (april_tag == 22){
            Inside_pose = new Pose(97, 61, Math.toRadians(0)); // secount clostest 22
            Outside_pose = new Pose(128, 61, Math.toRadians(0));
        }
        if (april_tag == 21){
            Inside_pose = new Pose(97, 85, Math.toRadians(0)); // third closest 23
            Outside_pose = new Pose(128, 85, Math.toRadians(0));
        }
        if (Inside_pose == null || Outside_pose == null){ // fall back incase error happens
            Inside_pose = new Pose(97, 37, Math.toRadians(0));
            Outside_pose = new Pose(128, 37, Math.toRadians(0));
        }

        start_to_fire_location = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(StartPose, FiringPose)
                )
                .setLinearHeadingInterpolation(StartPose.getHeading(), FiringPose.getHeading())
                .build();
        fire_location_to_ball_inside = follower.
                pathBuilder().
                addPath(new BezierLine(FiringPose, Inside_pose))
                .setLinearHeadingInterpolation(FiringPose.getHeading(), Inside_pose.getHeading())
                .build();

        ball_inside_to_ball_outside = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Inside_pose, Outside_pose)
                )
                .setConstantHeadingInterpolation(Inside_pose.getHeading())
                .build();

        ball_outside_to_fire_location = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(Outside_pose, ControlPointFiringLocation, FiringPose)
                )
                .setLinearHeadingInterpolation(Outside_pose.getHeading(), FiringPose.getHeading())
                .build();
        fire_location_to_park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(FiringPose, ParkPose)
                )
                .setLinearHeadingInterpolation(FiringPose.getHeading(), ParkPose.getHeading())
                .build();

    }
    private void telemetry(){
        AutoMainTelemetry();
        telemetry.addData("pathState", pathState);
        telemetry.addData("april tag", april_tag_value);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void init() {
        follower.setStartingPose(StartPose);
        // turns on all timlight stuff
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start(){
        //gets april tag value
        april_tag_value = limeLightData();
        //generates path based on april tag
        Paths_generation(april_tag_value);
    }
    //set up simple states
    public enum pathingState {START, First_Firing ,INSIDE, OUTSIDE, SECOND_FIRING, PARK, FINISHED}
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
                    if (LaunchBalls(3)) {
                        follower.followPath(fire_location_to_ball_inside);
                        pathState = pathingState.INSIDE;
                        decBot.intakeControl(50);
                    }
                }
                break;
            case INSIDE:
                if (!(follower.isBusy())) {
                    follower.followPath(ball_inside_to_ball_outside);
                    pathState = pathingState.OUTSIDE;
                }
                break;
            case OUTSIDE:
                if (!(follower.isBusy())) {
                    follower.followPath(ball_outside_to_fire_location);
                    pathState = pathingState.SECOND_FIRING;
                    decBot.intakeControl(0);
                }
                break;
            case SECOND_FIRING:
                if (!(follower.isBusy())) {
                    if (LaunchBalls(3)) {
                        follower.followPath(fire_location_to_park);
                        pathState = pathingState.PARK;
                    }
                }
                break;
            case PARK:
                if (!(follower.isBusy())) {
                    pathState = pathingState.FINISHED;
                }
                break;
            case FINISHED:
                break;
        }
    }
}

//I added coment
