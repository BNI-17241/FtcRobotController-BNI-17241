package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.AndrewState;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.StateAutoMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.MainContraints;

@Disabled
@Autonomous(name = "Really Dumb fire test", group = "Drive")
public class SimpleLaunchTest extends StateAutoMain {
//
    /**  Pedro Pathing Variables, Poses, Paths & States */
    public Follower follower;
    public Timer pathTimer, opmodeTimer;



    //set up simple states
    public enum pathingState {FIRE}
    public pathingState pathState = pathingState.FIRE;


    /**  Required OpMode Autonomous Control Methods  */

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        decBot.initRobot(hardwareMap);
        follower = MainContraints.createFollower(hardwareMap);
        follower.setStartingPose(BlueFarStartPose);
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        pathState = pathingState.FIRE;
    }


    @Override
    public void loop() {
        follower.update();
        AutoMainTelemetry();

        telemetry.update();


        switch (pathState) {
            case FIRE:
                //burnerLaunch(900, opmodeTimer.getElapsedTime(), 0);
        }
    }
}

