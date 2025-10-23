/**
 * Author: Jamie Acker
 * Purpose: Test out Machine States for Single and Multiple Loading into Fly Wheels
 * Note: Machine States do not use motor encoders, uses a timer)
 */

package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Acker;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;

@TeleOp(name = "DecodeBot Acker:Power", group = "Drive")
public class TeleOp17241_States extends OpMode {
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


    // Machine State Variables, Timers & Enums for Control of Feeder

    ElapsedTime timer = new ElapsedTime();

    // State Enum for Single Feeding
    public enum singleFeedStates {
        READY,
        START,
        PAUSE,
        STOP,
    }
    public singleFeedStates singleFeedState = singleFeedStates.READY;

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
    public multipleFeedStates multipleFeedState = multipleFeedStates.READY;

    // Driver Profiles
    public static final int PROFILE_1 = 1;  //User 1
    public static final int PROFILE_2 = 2; //user 2
    public int currentProfile = PROFILE_1;

    // Robot Constructor
    public DecodeBot decBot = new DecodeBot();

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
        flyWheelControl();
        feedStateController();
        feedStatesSingleLoad();
        feedStatesMultipleLoad();
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
        telemetry.addData("Left Fly Wheel: ", decBot.leftFlyWheel.getPower());
        telemetry.addData("Right Fly Wheel: ", decBot.rightFlyWheel.getPower());
        telemetry.update();
    }
    // ***** Helper Method for Speed Control
    public void speedControl() {
        if (gamepad1.dpad_up) {
            speedMultiply = 0.5;
        } else if (gamepad1.dpad_right) {
            speedMultiply = 0.75;
        } else if (gamepad1.dpad_down) {
            speedMultiply = 0.25;
        } else if (gamepad1.dpad_left) {
            speedMultiply = 1;
        }
    }


    //************ Control surface interfaces******************


    public void flyWheelControl(){

        if(gamepad1.x){
            decBot.flylaunch(true, .2);
        }
        else if(gamepad1.a){
            decBot.flylaunch(true, .4);
        }
        else if(gamepad1.b){
            decBot.flylaunch(true, .55);
        }

        if(gamepad1.right_bumper){
            decBot.flylaunch(false, 0);}
    }


    // Feed Controller using States
    public void feedStateController() {
        if (gamepad1.left_trigger > 0.5) {
            singleFeedState = singleFeedStates.START;
        }
        else if (gamepad1.right_trigger > 0.5)  {
            multipleFeedState = multipleFeedStates.START_1;
        }
    }

    // State Method for Single Feeding into fly wheels
    public void feedStatesSingleLoad() {
        switch (singleFeedState) {
            case START:
                decBot.feedArtifact( 1.0);
                timer.reset();
                singleFeedState = singleFeedStates.PAUSE;
                break;
            case PAUSE:
                if (timer.time() > 0.4) {
                    singleFeedState = singleFeedStates.STOP;
                }
                break;
            case STOP:
                decBot.feedArtifact( 0.0);
                singleFeedState = singleFeedStates.READY;
                break;
            case READY:
                break;
        }
    }

    // State Method for Multiple Feeding into fly wheels
    public void feedStatesMultipleLoad() {
        switch (multipleFeedState) {
            case START_1:
                decBot.feedArtifact( 1.0);
                timer.reset();
                multipleFeedState = multipleFeedStates.PAUSE_1;
                break;
            case PAUSE_1:
                if (timer.time() > 0.4) {
                    multipleFeedState = multipleFeedStates.STOP_1;
                }
                break;
            case STOP_1:
                decBot.feedArtifact( 0.0);
                timer.reset();
                multipleFeedState = multipleFeedStates.WAIT_1;
                break;
            case WAIT_1:
                if (timer.time() > 0.5) {
                    multipleFeedState = multipleFeedStates.START_2;
                }
                break;
            case START_2:
                decBot.feedArtifact( 1.0);
                timer.reset();
                multipleFeedState = multipleFeedStates.PAUSE_2;
                break;
            case PAUSE_2:
                if (timer.time() > 0.7) {
                    multipleFeedState = multipleFeedStates.STOP_2;
                }
                break;
            case STOP_2:
                decBot.feedArtifact( 0.0);
                timer.reset();
                multipleFeedState = multipleFeedStates.WAIT_2;
                break;
            case WAIT_2:
                if (timer.time() > 0.5) {
                    multipleFeedState = multipleFeedStates.START_3;
                }
                break;
            case START_3:
                decBot.feedArtifact( 1.0);
                timer.reset();
                multipleFeedState = multipleFeedStates.PAUSE_3;
                break;
            case PAUSE_3:
                if (timer.time() > 0.7) {
                    multipleFeedState = multipleFeedStates.STOP_3;
                }
                break;
            case STOP_3:
                decBot.feedArtifact( 0.0);
                timer.reset();
                multipleFeedState = multipleFeedStates.READY;
                break;
            case READY:
                break;
        }
    }


}