package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.StateAuto.Red;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoTemplates.StateHumanMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AutoTemplates.StateVarableMain;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.MainContraints;

@Autonomous(name = "Red Human Two Spike State", group = "Drive")
public class RedHumanTwoSpikeState extends StateHumanMain {

    //When to go to park as failsafe (0-30 seconds from start, recommended 25)


    @Override
    public void init() {

        StartingPose = RedFarStartPose;
        //Shoot Pose
        ShootingPose = RedFarShootPose;
        //Park Pose
        ParkingPose = RedFarParkPose;

        //Delay before initial movement (ms)
        startDelay = 0;


        maxTimeBreakout = 29 * 1000;
        targetVelocity = 1400;
        intakeSpeed = 1;
        intakeMoveSpeed = 0.4;
        variance = 20;
        xAutoOffset = 3;//-3
        pathOffset = 0;


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        decBot.initRobot(hardwareMap);
        follower = MainContraints.createFollower(hardwareMap);
        redPathGen();
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
