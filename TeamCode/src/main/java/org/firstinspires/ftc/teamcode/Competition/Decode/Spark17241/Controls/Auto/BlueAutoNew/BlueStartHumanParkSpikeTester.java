package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.BlueAutoNew;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.StateAutoMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.MainContraints;


@Autonomous(name = "Blue:Start Human:Park Spike State Test", group = "Drive")
public class BlueStartHumanParkSpikeTester extends StateAutoMain {
//
    /**  Pedro Pathing Variables, Poses, Paths & States */
    public Follower follower;
    public Timer pathTimer, opmodeTimer;

    //Delay before intial movement (ms)
    public final float startDelay = 3000;
    //Delay at goal after firing (ms)
    public final float fireDelay  = 5000;


    //Help view elapsed time on wait cases
    public float delayStartTime = 0;

    public Path scorePreload;

    protected PathChain path0;
    protected PathChain path1;

    //set up simple states
    public enum pathingState {STARTDELAY, START, FIRINGDELAY, FIRING, PARK, END}
    public pathingState pathState = pathingState.START;

    public void pathGen(){
        path0 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueFarStartPose, BlueMidShootPose)
                )
                .setLinearHeadingInterpolation(BlueFarStartPose.getHeading(), BlueMidShootPose.getHeading())
                .build();

        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(BlueMidShootPose, BlueFarParkPose)
                )
                .setLinearHeadingInterpolation(BlueMidShootPose.getHeading(), BlueFarParkPose.getHeading())
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
    }


    @Override
    public void loop() {
        follower.update();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Current time : ", opmodeTimer.getElapsedTime());
        telemetry.addData("Wait time : ", delayStartTime);

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
                follower.followPath(path0);
                pathState = pathingState.FIRING;
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
                    follower.followPath(path1);
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

