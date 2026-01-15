package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto;
import com.pedropathing.paths.Path;
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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.RedAlliance.RedStartHumanFarLaunch;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;

@Autonomous(name = "Red:Start Human:Far Launch", group = "Drive")
public class padro_test extends OpMode {

    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    protected Limelight3A limelight;

    public Pose Inside_pose;
    public Pose Outside_pose;
    public final Pose StartPose = new Pose(42, 8.5, Math.toRadians(0));
    public final Pose FiringPose = new Pose(58, 86, Math.toRadians(135));
    public final Pose ParkPose = new Pose(45, 12, Math.toRadians(0));


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
        if (april_tag == 21){
             final Pose Inside_pose = new Pose(48, 81, Math.toRadians(180)); // closest to human 21
             final Pose Outide_pose = new Pose(32, 81, Math.toRadians(180));
        }
        if (april_tag == 22){
            final Pose Inside_pose = new Pose(48, 57.5, Math.toRadians(180)); // secount clostest 22
            final Pose Outide_pose = new Pose(32, 57.5, Math.toRadians(180));
        }
        if (april_tag == 23){
            final Pose Inside_pose = new Pose(48, 34, Math.toRadians(180)); // third closest 23
            final Pose Outide_pose = new Pose(32, 34, Math.toRadians(180));
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
        limelight.start();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start(){
        april_tag_value = limeLightData();
        Paths_generation(april_tag_value);
    }

    public enum pathingState {START, INSIDE, OUTSIDE, FIRING, PARK, FINISHED}
    public pathingState pathState = pathingState.START;

    @Override
    public void loop() {
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
            case FINISHED:
                break;
        }
    }
}
