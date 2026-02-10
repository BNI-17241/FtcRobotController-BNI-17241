package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.StateDecodeBot;

import java.util.ArrayList;
import java.util.List;

//TeleOp for Garry
@TeleOp(name = "Garry Teleop", group = "Drive")
public class GarryTeleop extends OpMode {

    protected  Limelight3A limelight;


    // Drivetrain Variables
    protected double leftStickYVal;
    protected double leftStickXVal;
    protected double rightStickYVal;
    protected double rightStickXVal;
    protected double frontLeftSpeed;
    protected double frontRightSpeed;
    protected double rearLeftSpeed;
    protected double rearRightSpeed;
    protected double powerThreshold;

    protected double moveSpeedMultiply = 0.75;

    // Drive Profile Control Variables
    protected  static final int PROFILE_1 = 1;  //User 1
    protected  static final int PROFILE_2 = 2; //user 2
    protected  int currentProfile = PROFILE_1;

    //Limelight Cam data
    protected  LLResult result;

    protected double autoTargetRotation = 0;
    protected double autoTargetSpeed = .4;
    protected double autoYawVariation = 1;

    protected boolean isIntaking = false;

    protected boolean isTracking = false;

    //Velocity of the Launching wheels
    protected double targetVelocity;

    protected double tolerance = 50;

    protected double min_velocity_drop = 50; // threshold for detecting ball contact
    protected List<Double> previousShotVelocityL = new ArrayList<>();

    protected boolean hasStartedAutoLaunch = false;
    protected boolean hasStartedFeeding = false;
    protected boolean hasReleased = true;

    protected double transferServoSpeed = 1;
    protected double intakeMotorSpeed = 1;

    // Instantiation of Robot using Robot Class Constructor
    public StateDecodeBot decBot = new StateDecodeBot();


    //Shooting variables
    protected boolean isLaunching = false;


    @Override
    public void init() {
        decBot.initRobot(hardwareMap);
       // decBot.imu.resetYaw();// REV
       //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(0);
      // limelight.start();
    }

    @Override
    public void loop() {
        driverOneInput();
        driverTwoInput();
        setFlywheelSpeed();
        //limeLightData();
        //autoTarget();
        robotCentricDrive();
        //LEDDriver();
        telemetryOutput();
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
                // leftStickXVal controls strafing, and rightStickXVal controls rotation.
                frontLeftSpeed = leftStickYVal + rightStickXVal + leftStickXVal;    // Vertical + Rotation + Staffing
                frontRightSpeed = leftStickYVal - rightStickXVal - leftStickXVal;   // Vertical - Rotation - Strafing(sign in front is the way the motor is turning in relation to the others)
                rearLeftSpeed = leftStickYVal - rightStickXVal + leftStickXVal;
                rearRightSpeed = leftStickYVal + rightStickXVal - leftStickXVal;

                break;
            case PROFILE_2:
                //leftStickXVal controls rotation, and rightStickXVal controls strafing.
                frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal; // Vertical + Rotation + Staffing
                frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;// Vertical - Rotation - Strafing
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
        setMotorPower(decBot.frontLeftMotor, frontLeftSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(decBot.frontRightMotor, frontRightSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(decBot.rearLeftMotor, rearLeftSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(decBot.rearRightMotor, rearRightSpeed, powerThreshold, moveSpeedMultiply);
    }



    //********* Firing Control ****************************
    public boolean canLaunch(double selected_speed, double tolerance) {
        double upper_tolerance = Math.max(0, selected_speed + tolerance);
        double lower_tolerance = Math.max(0, selected_speed - tolerance);
        double currentVelocity = getCurrentVelocity();

        return currentVelocity >= lower_tolerance && currentVelocity <= upper_tolerance;
    }

    public void setFlywheelSpeed(){
        //Launch if is allowed to
        decBot.flylaunch(isLaunching ? targetVelocity : 0);
    }
    // ***** Manual Feeder Wheel Controller
    public void driverOneInput() {
        //Change ground move speed
        if (gamepad1.dpad_right) {
            moveSpeedMultiply = 0.5;
        } else if (gamepad1.dpad_left) {
            moveSpeedMultiply = 0.75;
        } else if (gamepad1.dpad_up) {
            moveSpeedMultiply = 0.25;
        } else if (gamepad1.dpad_down) {
            moveSpeedMultiply = 1;
        }

        if(gamepad1.left_bumper){
            isIntaking = true;
        }
        if(gamepad1.right_bumper){
            isIntaking = false;
        }

        if(isIntaking){
            decBot.intakeControl(gamepad1.left_trigger > 0.5 ? -intakeMotorSpeed : intakeMotorSpeed);
        }
        else{
            decBot.intakeControl(0);
        }
    }
    public void driverTwoInput(){
        if(gamepad2.right_bumper){
            decBot.transferSpeedCon(transferServoSpeed);
        }
        else if(gamepad2.left_bumper){
            decBot.transferSpeedCon(-transferServoSpeed);
        }
        else{
            decBot.transferSpeedCon(0);
        }

        if (gamepad1.x) { // Square
            // NEAR preset
            targetVelocity = 2500;
        }
        if (gamepad1.a) { // X
            targetVelocity = 1259;
        }
        if (gamepad1.b) { // Circle
            targetVelocity = 1368;
        }
        if (gamepad1.y) { // Triangle
            targetVelocity = 1520;
        }
        if(gamepad1.right_stick_button)
        {
            targetVelocity = 1550;
        }

//        if (gamepad2.dpad_up)     { targetVelocity += 1; }
//        if (gamepad2.dpad_down)   { targetVelocity -= 1; }

        if (gamepad1.left_trigger > 0.5){
            isLaunching = false;
        }
        if (gamepad1.right_trigger > 0.5){
            isLaunching = true;
        }

    }

    //Auto Correction
    public void limeLightData() {
        result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
    }


    // ***** Helper Method for Telemetry
    public void telemetryOutput() {
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addLine("-------------------------------------");
//        telemetry.addData("Is Tracking: ", isTracking);
//        telemetry.addData("Is Launching", isLaunching);
        telemetry.update();
    }
    //******** Helper Functions **************************
    private double getCurrentVelocity() {
        // Averages front and back motor velocity
        return (decBot.launchFrontMotor.getVelocity() + decBot.launchBackMotor.getVelocity()) / 2.0;
    }
    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }
    // ****** Led Controller
    public void LEDDriver() {
        decBot.LEDCon(6);
    }
}