package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.BlueAutoNew.Far;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoTemplates.StateVarableMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.MainContraints;

@Autonomous(name = "Sub Blue Human Med 1 state", group = "Drive")
public class StateSubVarableHumanBlueMed1 extends StateVarableMain {

    //When to go to park as failsafe (0-30 seconds from start, recommended 25)


    @Override
    public void init() {
        StartingPose = BlueFarStartPose;
        //Shoot Pose
        ShootingPose = BlueMidShootPose;
        //Park Pose
        ParkingPose = BlueFarParkPose;

        //Optional Pose for shooting after Third Spike
        ThirdShootPose = BlueMidShootPose;

        //Delay before initial movement (ms)
        startDelay = 0;

        //How many spikes are needed? 0-3
        spikeAmount = 1;
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
        AtoCIntake = true;
        maxTimeBreakout = 29 * 1000;
        targetVelocity = 1260;
        intakeSpeed = 1;
        intakeMoveSpeed = 0.4;
        variance = 90;




        pathTimer = new Timer();
        opmodeTimer = new Timer();
        decBot.initRobot(hardwareMap);
        follower = MainContraints.createFollower(hardwareMap);
        pathGen();
        follower.setStartingPose(StartingPose);
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
        stateVarableMainTelemetry();
        telemetry.update();


        autoStateLoop();
    }

}
