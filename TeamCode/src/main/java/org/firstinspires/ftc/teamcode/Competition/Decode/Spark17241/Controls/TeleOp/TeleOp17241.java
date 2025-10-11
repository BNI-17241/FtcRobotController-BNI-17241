package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;

@TeleOp(name = "DecodeBot TeleOp", group = "Drive")
public class TeleOp17241 extends OpMode {
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

    boolean flywheelIsOn = false;
    double flywheelSpeed = 0.5;
    final double SPEED_INCREMENT = 0.01;
    boolean previousDpadUp = false;


    boolean feedWheelIsOn = false;
    boolean previousDpadDown = false;
    boolean feedIsOn = false;

// I andrew decree that I am very very very very flipping short

    private static final int PROFILE_1 = 1;  //User 1
    private static final int PROFILE_2 = 2; //user 2
    private int currentProfile = PROFILE_1;

    public DecodeBot decBot = new DecodeBot();

    //public Pinpoint odo = new Pinpoint();

    //double botHeading = decBot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    @Override
    public void init() {
        decBot.initRobot(hardwareMap);
        //odo.initPinpoint(hardwareMap);

        // resetHeading();                       // PINPOINT
        decBot.imu.resetYaw();                   // REV
    }



    @Override
    public void loop() {
        speedControl();
        telemetryOutput();
        RobotCentricDrive();
        FlyWheelControl();
        feed();

        //fieldCentricDrive();
        imuStart();
        //driveCases();
        //transferControl();
    }





//    // ********* TeleOp Control Methods **************

    // ****** Helper method to set Motor Power
    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }
    public void imuStart(){
        if(gamepad1.options){
            decBot.imu.resetYaw();
        }
    }

    //*********  Driver 1 and Driver 2 Control Methods


    // ********  Legacy Drive Control Methods
//
//    public void fieldCentricDrive() {
//        if (gamepad1.options) {
//            decBot.imu.resetYaw();
//        }
//        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;
//
//        // This button choice was made so that it is hard to hit on accident,
//        // it can be freely changed based on preference.
//        // The equivalent button is start on Xbox-style controllers.
//        if (gamepad1.options) {
//            decBot.imu.resetYaw();
//        }
//
//        double botHeading = decBot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        double frontLeftPower = (rotY + rotX + rx) / denominator;
//        double backLeftPower = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower = (rotY + rotX - rx) / denominator;
//
//        decBot.frontLeftMotor.setPower(frontLeftPower);
//        decBot.rearLeftMotor.setPower(backLeftPower);
//        decBot.frontRightMotor.setPower(frontRightPower);
//        decBot.rearRightMotor.setPower(backRightPower);
//    }


    // Robot Centric Drive Method
    public void RobotCentricDrive() {

        leftStickYVal = gamepad1.left_stick_y;
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
       // telemetry.addData("Heading: ",  getHeading());
       // telemetry.addData("Current X Pos: ", odo.getPosition().getX(DistanceUnit.INCH));
       // telemetry.addData("Current Y Pos: ", odo.getPosition().getY(DistanceUnit.INCH));
        //telemetry.addData("Hue Value: ", sensor.hsvValues[0]);
        telemetry.addData("Right Odo", decBot.rightEncoder.getCurrentPosition());
        telemetry.addData("Center Odo", decBot.centerEncoder.getCurrentPosition());
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


    public void FlyWheelControl(){
        if (gamepad1.dpad_up && !previousDpadUp) {
            flywheelIsOn = !flywheelIsOn;
        }
        previousDpadUp = gamepad1.dpad_up;
        if (flywheelIsOn) {
            decBot.flylaunch(true, flywheelSpeed);
        } else {
            decBot.flylaunch(false, 0);
        }
        if (gamepad1.dpad_right) {
            if (flywheelSpeed <= 1) {
                flywheelSpeed += SPEED_INCREMENT;
            }
        }
        if (gamepad1.dpad_left) {
            if (flywheelSpeed >= 0) {
                flywheelSpeed -= SPEED_INCREMENT;
            }
        }
    }
    public void feed(){
        if (gamepad1.dpad_down && !previousDpadDown) {
            feedIsOn = !feedWheelIsOn;
        }
        previousDpadDown = gamepad1.dpad_down;
        feedWheelIsOn = true;
        if (feedWheelIsOn) {
            decBot.feedArtifact(true, 0.2);
        } else {
            decBot.feedArtifact(false, 0);
        }
    }




}