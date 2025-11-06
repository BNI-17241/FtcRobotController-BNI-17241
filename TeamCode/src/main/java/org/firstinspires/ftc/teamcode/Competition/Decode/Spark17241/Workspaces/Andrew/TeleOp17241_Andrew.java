package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;
@Disabled
@TeleOp(name = "DecodeBot Andrew", group = "Lab")
public class TeleOp17241_Andrew extends OpMode {
    public double leftStickYVal;
    public double leftStickXVal;
    public double rightStickYVal;
    public double rightStickXVal;

    public double frontLeftSpeed;
    public double frontRightSpeed;
    public double rearLeftSpeed;
    public double rearRightSpeed;

    public double powerThreshold;
    public double speedMultiply = 0.75;

    public int ledCycle = 0;


    // Flywheel & Feed Variables
    public double targetVelocity = 1600;
    public double feedingDuration = 0.4;
    public double feedingDurationLong = 0.7;


    public int flyPos;
    public double motorRevolutions;

    public double angle;
    public double angleNormalized;




    // Machine State Variables, Timers & Enums for Control of Feeder

    ElapsedTime timer = new ElapsedTime();

    // State Enum for Single Feeding
    public enum singleFeedStates {
        READY,
        START,
        PAUSE,
        STOP,
    }

    public org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.singleFeedStates singleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.singleFeedStates.READY;

    // State Enum for Multiple Feeding
    public enum multipleFeedStates {
        READY,
        // First Feed into Fly Wheels
        START_1,
        PAUSE_1,
        STOP_1,
        WAIT_1,
        // Second Feed into Fly Wheels
        START_2,
        PAUSE_2,
        STOP_2,
        WAIT_2,
        // Third Feed into Fly Wheels
        START_3,
        PAUSE_3,
        STOP_3,
    }

    public org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.READY;

    // Driver Profiles
    public static final int PROFILE_1 = 1;  //User 1
    public static final int PROFILE_2 = 2; //user 2
    public int currentProfile = PROFILE_1;

    // Robot Constructor
    public DecodeBot_Andrew decBot = new DecodeBot_Andrew();

    // Initialization Method for Hardware
    @Override
    public void init() {
        decBot.initRobot(hardwareMap);
    }

    // Play Loop
    @Override
    public void loop() {
        // Driving Controls
        speedControl();
        robotCentricDrive();
        //Launching Controls
        beltControl();
        flyWheelControl();
        //flyEncoderData();
        feedStateController();
       // feedStatesSingleLoad();
        //feedStatesMultipleLoad();
        //LED
        LEDDriver();
        // Helper Controls
        telemetryOutput();
    }

    // ********* TeleOp Control Methods **************

    // Helper method to set Motor Power
    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }

    // Robot Centric Drive Method
    public void robotCentricDrive() {
        //reverse drive
        leftStickYVal = -gamepad1.left_stick_y;

        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
        rightStickYVal = gamepad1.right_stick_y;
        rightStickYVal = Range.clip(rightStickYVal, -1, 1);

        leftStickXVal = gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = gamepad1.right_stick_x;
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);

        switch (currentProfile) {
            // Name of Driver using Profile 1
            case PROFILE_1:
                // leftStickXVal controls rotation, and rightStickXVal controls strafing.
                frontLeftSpeed = leftStickYVal + rightStickXVal + leftStickXVal;    // Vertical + Rotation + Staffing
                frontRightSpeed = leftStickYVal - rightStickXVal - leftStickXVal;   // Vertical - Rotation - Strafing(sign in front is the way the motor is turning in relation to the others)
                rearLeftSpeed = leftStickYVal - rightStickXVal + leftStickXVal;
                rearRightSpeed = leftStickYVal + rightStickXVal - leftStickXVal;
                break;
            case PROFILE_2:
                // This is copy of profile one
                frontLeftSpeed = leftStickYVal + rightStickXVal + leftStickXVal;    // Vertical + Rotation + Staffing
                frontRightSpeed = leftStickYVal - rightStickXVal - leftStickXVal;   // Vertical - Rotation - Strafing(sign in front is the way the motor is turning in relation to the others)
                rearLeftSpeed = leftStickYVal - rightStickXVal + leftStickXVal;
                rearRightSpeed = leftStickYVal + rightStickXVal - leftStickXVal;

                // Default Driver Profile
            default:
                // stops robot
                frontLeftSpeed = 0;
                frontRightSpeed = 0;
                rearLeftSpeed = 0;
                rearRightSpeed = 0;
                break;
        }

        // Clipping motor speeds to [-1, 1]
        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);
        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

        // Setting motor powers (with threshold check)
        setMotorPower(decBot.frontLeftMotor, frontLeftSpeed, powerThreshold, speedMultiply);
        setMotorPower(decBot.frontRightMotor, frontRightSpeed, powerThreshold, speedMultiply);
        setMotorPower(decBot.rearLeftMotor, rearLeftSpeed, powerThreshold, speedMultiply);
        setMotorPower(decBot.rearRightMotor, rearRightSpeed, powerThreshold, speedMultiply);
    }

    // ***** Helper Method for Telemetry
    public void telemetryOutput() {
        telemetry.addData("Single Feed State: ", singleFeedState);
        telemetry.addData("Multiple Feed State: ", multipleFeedState);
        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addData("Left Fly Wheel Velocity: ", decBot.leftFlyWheel.getVelocity());
        telemetry.addData("Right Fly Wheel Velocity: ", decBot.rightFlyWheel.getVelocity());
        telemetry.update();
    }

    // ***** Helper Method for Speed Control
    public void speedControl() {
        /*if (gamepad1.dpad_up) {
            speedMultiply = 0.5;
        } else if (gamepad1.dpad_right) {
            speedMultiply = 0.75;
        } else if (gamepad1.dpad_down) {
            speedMultiply = 0.25;
        } else if (gamepad1.dpad_left) {
            speedMultiply = 1;
        }*/
        speedMultiply = .75;
    }

    //****************** Flywheel Encoder Data ****************
    public void flyEncoderData()
    {
        flyPos = decBot.leftFlyWheel.getCurrentPosition();
    }

    //*****************LED Control**********************
    public void LEDDriver()
    {
        if(gamepad1.left_stick_button){ledCycle += 1;}
        if(ledCycle > 6){ledCycle = 0;}
        decBot.LEDCon(ledCycle);
    }


    //***************Run 360 Servo********************

    public void beltControl()
    {/*
        if(gamepad1.dpad_up)
        {
            decBot.runBelt(1);
        }*/
    }




    //************ Control surface interfaces******************


    public void flyWheelControl() {

        if (gamepad2.x) { targetVelocity = 1000; }
        if (gamepad2.a) { targetVelocity = 1100; }
        if (gamepad2.b) { targetVelocity = 1254; }
        if (gamepad2.dpad_up) targetVelocity += 1;
        if (gamepad2.dpad_down) targetVelocity -= 1;
        if (gamepad2.right_bumper) { targetVelocity = 0; }
        if (gamepad2.left_bumper) { targetVelocity = -500; }

        decBot.flylaunch(targetVelocity);
    }




    // Feed Controller using States
    public void feedStateController() {
        if (gamepad2.left_trigger > 0.5) {
            decBot.feedArtifact(1.0);
            //singleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.singleFeedStates.START;
        } else if (gamepad2.right_trigger > 0.5) {
            decBot.feedArtifact(-1.0);
        } else {
            decBot.feedArtifact(0);
        }

        /*else if (gamepad1.right_trigger > 0.5) {
            multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.START_1;
        }*/
    }

    // State Method for Single Feeding into fly wheels
    public void feedStatesSingleLoad() {
        switch (singleFeedState) {
            case START:
                decBot.feedArtifact(1.0);
                timer.reset();
                singleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.singleFeedStates.PAUSE;
                break;
            case PAUSE:
                if (timer.time() > 0.3) {
                    singleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.singleFeedStates.STOP;
                }
                break;
            case STOP:
                decBot.feedArtifact(0.0);
                singleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.singleFeedStates.READY;
                break;
            case READY:
                break;
        }
    }

    // State Method for Multiple Feeding into fly wheels
    public void feedStatesMultipleLoad() {
        switch (multipleFeedState) {
            case START_1:
                decBot.feedArtifact(1.0);
                timer.reset();
                multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.PAUSE_1;
                break;
            case PAUSE_1:
                if (timer.time() > 0.3) {
                    multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.STOP_1;
                }
                break;
            case STOP_1:
                decBot.feedArtifact(0.0);
                timer.reset();
                multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.WAIT_1;
                break;
            case WAIT_1:
                if (timer.time() > .5) {
                    multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.START_2;
                }
                break;
            case START_2:
                decBot.feedArtifact(1.0);
                timer.reset();
                multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.PAUSE_2;
                break;
            case PAUSE_2:
                if (timer.time() > 0.3) {
                    multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.STOP_2;
                }
                break;
            case STOP_2:
                decBot.feedArtifact(0.0);
                timer.reset();
                multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.WAIT_2;
                break;
            case WAIT_2:
                if (timer.time() > 0.5) {
                    multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.START_3;
                }
                break;
            case START_3:
                decBot.feedArtifact(1.0);
                timer.reset();
                multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.PAUSE_3;
                break;
            case PAUSE_3:
                if (timer.time() > 0.3) {
                    multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.STOP_3;
                }
                break;
            case STOP_3:
                decBot.feedArtifact(0.0);
                timer.reset();
                multipleFeedState = org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew.TeleOp17241_Andrew.multipleFeedStates.READY;
                break;
            case READY:
                break;
        }
    }


}