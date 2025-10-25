package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;

@TeleOp(name = "DecodeBot Comp", group = "Drive")
public class TeleOp17241 extends OpMode {

    // Drivetrain Variables
    double leftStickYVal;
    double leftStickXVal;
    double rightStickYVal;
    double rightStickXVal;
    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;
    double powerThreshold;
    double speedMultiply = 0.75;

    // Flywheel & Feed Wheel Variables
    public double targetVelocity = 0;
    public double feedingDuration = 0.4;
    public double feedingDurationLong = 0.7;

    // Drive Profile Control Variables
    public static final int PROFILE_1 = 1;  //User 1
    public static final int PROFILE_2 = 2; //user 2
    public int currentProfile = PROFILE_1;


    // Instantiation of Robot using Robot Class Constructor
    public DecodeBot decBot = new DecodeBot();


    @Override
    public void init() {
        decBot.initRobot(hardwareMap);
        decBot.imu.resetYaw();                   // REV
    }


    @Override
    public void loop() {
        speedControl();
        telemetryOutput();
        robotCentricDrive();
        flyWheelControl();
        feedWheelManualControl();
    }



    //*********  Driver 1 Control Methods *****************


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
                //leftStickXVal controls strafing, and rightStickXVal controls rotation.
                frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
                frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
                rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
                rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
                break;

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

    //*********  Driver 2 Control Methods *****************


    // Fly Wheel Control
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

    // ***** Manual Feeder Wheel Controller
    public void feedWheelManualControl() {
        if (gamepad2.left_trigger > 0.5) {
            decBot.feedArtifact(1.0);
        }
        else if (gamepad2.right_trigger > 0.5) {
            decBot.feedArtifact(-1.0);
        }
        else {
            decBot.feedArtifact(0);
        }

    }


    // ***** Helper Method for Telemetry
    public void telemetryOutput() {
        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addData("Left Fly Wheel Velocity: ", decBot.leftFlyWheel.getVelocity());
        telemetry.addData("Right Fly Wheel Velocity: ", decBot.rightFlyWheel.getVelocity());
        telemetry.update();
    }


    // ****** Helper method to set Motor Power
    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }




}