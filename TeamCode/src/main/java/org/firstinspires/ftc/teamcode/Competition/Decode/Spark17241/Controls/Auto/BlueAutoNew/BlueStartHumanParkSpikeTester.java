package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.BlueAutoNew;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.Old.BlueAlliance.BlueStartHumanParkSpike;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;



@Autonomous(name = "Blue:Start Human:Park Spike State Test", group = "Drive")
public class BlueStartHumanParkSpikeTester extends OpMode {
//
    /**  Pedro Pathing Variables, Poses, Paths & States */
    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    public final Pose startPose = new Pose(44, 8, Math.toRadians(90));     // Red Far Launch Zone start
    public final Pose scorePose = new Pose(59, 81, Math.toRadians(133));    // Red goal scoring pose // 80 x 80
    public final Pose parkPose = new Pose(43, 12, Math.toRadians(90)); // Red Home (park)

    //Delay before intial movement (ms)
    public final float startDelay = 3000;
    //Delay at goal after firing (ms)
    public final float fireDelay  = 5000;


    //Help view elapsed time on wait cases
    public float delayStartTime = 0;

    public Path scorePreload;

    protected PathChain start_to_fire_location;
    protected PathChain fire_location_to_park;

    //set up simple states
    public enum pathingState {STARTDELAY, START, FIRINGDELAY, FIRING, PARK, END}
    public pathingState pathState = pathingState.START;

    public void pathGen(){
        start_to_fire_location = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, scorePose)
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        fire_location_to_park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, parkPose)
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    /**  Required OpMode Autonomous Control Methods  */

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        pathGen();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        pathState = pathingState.STARTDELAY;
        //Get the time of the starting delay
        delayStartTime = opmodeTimer.getElapsedTime();
    }


    @Override
    public void loop() {
        follower.update();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Current state :", pathState);
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
                //Move to the firing location
                follower.followPath(start_to_fire_location);
                pathState = pathingState.FIRING;
                break;

            case FIRING:
                if (!(follower.isBusy())) {
                    follower.followPath(fire_location_to_park);
                    pathState = pathingState.FIRINGDELAY;
                    //Reset delay
                    delayStartTime = opmodeTimer.getElapsedTime();
                }
                break;

            case FIRINGDELAY:
                //Check if wait has been fulfilled
                if(delayStartTime <= opmodeTimer.getElapsedTime() - fireDelay) {
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

