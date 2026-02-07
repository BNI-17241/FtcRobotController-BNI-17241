package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.StateAuto.Blue;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoTemplates.StateVarableMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.MainContraints;

@Autonomous(name = "Blue Far Two Spike State", group = "Drive")
public class BlueFarTwoSpikeState extends StateVarableMain {

    //When to go to park as failsafe (0-30 seconds from start, recommended 25)


    @Override
    public void init() {

        StartingPose = BlueFarStartPose;
        //Shoot Pose
        ShootingPose = BlueFarShootPose;
        //Park Pose
        ParkingPose = BlueFarParkPose;

        //Optional Pose for shooting after Third Spike
        ThirdShootPose = ShootingPose;

        //Delay before initial movement (ms)
        startDelay = 0;

        //How many spikes are needed? 0-3
        spikeAmount = 2;
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
        targetVelocity = 1400;
        intakeSpeed = 1;
        intakeMoveSpeed = 0.4;
        variance = 20;
        xAutoOffset = -3;//-3
        pathOffset = 0;// Math.PI / 18;
        farLaunch = true;


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        decBot.initRobot(hardwareMap);
        follower = MainContraints.createFollower(hardwareMap);

        //Change for alliance
        bluePathGen();

        follower.setStartingPose(StartingPose);


        limelightInit();

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
        telemetry.addData("Velocity: ", getCurrentVelocity());
        //AutoMainTelemetry();
        //stateVarableMainTelemetry();
        telemetry.update();


        autoStateLoop();
    }

}
