package org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.BlueLeftPark.GyroDriveLeft;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.CompetionBot;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.StraferBot;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.Test.PID.CompetitionPIDTest.AutoTargetZone;
@Disabled
@Autonomous (name = "LEFT AUTO")
public class AutoLeftStack extends LeftLowConeStack {

    public CompetionBot Bot = new CompetionBot();

    public StraferBot BotStrafer = new StraferBot();

    public AutoTargetZone targetZone = null;

    public boolean isCompetition = true;


    @Override
    public void runOpMode() throws InterruptedException {

        if (isCompetition == true) {
            Bot.initRobot(hardwareMap);

            Bot.setLinearOp(this);

        } else {

            BotStrafer.initRobot(hardwareMap);

            BotStrafer.setLinearOp(this);

        }

        // Initialize WebCam and Create Image Processing Pipeline
        initializePipeline();

        //targetZone = AutoTargetZone.A;
        Bot.openGrabberArms();

        telemetry.addLine("WAITING FOR START >");
        telemetry.addData("TARGET ZONE: ", TargetZone);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            Bot.turretPlatform.setPower(0);

//            Bot.turretPlatform.setPower(.3);
//            Bot.turretPlatform.setTargetPosition(0);
//            Bot.turretPlatform.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Find Tags During the Init Loop
            while (!isStarted() && !isStopRequested()) {
                findTag();
                sleep(20);
            }

            //detect tags in auto main

            detectTags();

            if (isCompetition == true) {

                sleep(sleepTime);

                Bot.closeGrabberArms();

                sleep(1000);

            }

            //targetZone = DetectSleaveImage(Bot);

            telemetry.addData("TARGET ZONE: ", TargetZone);
            telemetry.update();

            sleep(1000);

            if (isCompetition == true) {

                scoreHigh(Bot);
                sleep(1000);
                parkZone (Bot, TargetZone);
                sleep(sleepTime);

            }

            idle();
            requestOpModeStop();
        }
    }
}
