package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;


@Autonomous(name = "Multi test", group = "Drive")
public class PadroLimLightTestOz extends OpMode {

    public Follower follower;
    public Timer pathTimer, opmodeTimer;


    protected Limelight3A limelight;

    public Pose Inside_pose;
    public Pose Outside_pose;
    // SIMPLE POSES
    public final Pose StartPose = new Pose(42, 8.5, Math.toRadians(90));
    public final Pose FiringPose = new Pose(58, 86, Math.toRadians(135));
    public final Pose ParkPose = new Pose(45, 18, Math.toRadians(90));

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


    protected PathChain start_to_ball_inside;
    protected PathChain ball_inside_to_ball_outside;
    protected PathChain ball_outside_to_fire_location;
    protected PathChain fire_location_to_park; //why is bro reading my code 

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

        if (april_tag == 21){
            Inside_pose = new Pose(48, 36, Math.toRadians(180)); // closest to human 21
            Outside_pose = new Pose(16, 36, Math.toRadians(180));
        }
        if (april_tag == 22){
            Inside_pose = new Pose(48, 61, Math.toRadians(180)); // secount clostest 22
            Outside_pose = new Pose(16, 61, Math.toRadians(180));
        }
        if (april_tag == 23){
            Inside_pose = new Pose(48, 85, Math.toRadians(180)); // third closest 23
            Outside_pose = new Pose(16, 85, Math.toRadians(180));
        }
        if (Inside_pose == null || Outside_pose == null){ // fall back incase error happens
            Inside_pose = new Pose(48, 37, Math.toRadians(180));
            Outside_pose = new Pose(16, 37, Math.toRadians(180));
        }

        start_to_ball_inside = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(StartPose, Inside_pose)
                )
                .setLinearHeadingInterpolation(StartPose.getHeading(), Inside_pose.getHeading())
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
                        new BezierLine(Outside_pose, FiringPose)
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

    @Override
    public void init() {
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
    public enum pathingState {START, INSIDE, OUTSIDE, FIRING, PARK, FINISHED}
    public pathingState pathState = pathingState.START;

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
        //telemetry.addData("Pose", Pose)

        // very simple movment test
        switch (pathState) {
            case START:
                follower.followPath(start_to_ball_inside);
                pathState = pathingState.INSIDE;
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
                    pathState = pathingState.FIRING;
                }
                break;
            case FIRING:
                if (!(follower.isBusy())) {
                    follower.followPath(fire_location_to_park);
                    pathState = pathingState.PARK;
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
