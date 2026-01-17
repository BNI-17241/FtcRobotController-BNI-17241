package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.DecodeBot_One__Wheel_Launch;

//

@TeleOp(name = "Decode_One_Wheel_Launch_TeleOP", group = "Drive")
public class Decode_One_Wheel_Launch_TeleOP extends OpMode {

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


    //Velocity of the Launching wheels
    protected double targetVelocity;

    protected double tolerance = 50;


    // Instantiation of Robot using Robot Class Constructor
    public DecodeBot_One__Wheel_Launch decBot = new DecodeBot_One__Wheel_Launch();


    @Override
    public void init() {
        decBot.initRobot(hardwareMap);
        decBot.imu.resetYaw();// REV
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void loop() {
        firingControl();
        basics();
    }


    protected double min_velocity_drop = 50; // threshold for detecting ball contact
    protected List<Double> previousShotVelocityL = new ArrayList<>();

    protected boolean hasStartedAutoLaunch = false;
    protected boolean hasStartedFeeding = false;
    protected boolean hasReleased = true;

    private double getCurrentVelocity() {
        // Averages front and back motor velocity
        return (decBot.launchFrontMotor.getVelocity() + decBot.launchBackMotor.getVelocity()) / 2.0;
    }

    public boolean canLaunch(double selected_speed, double tolerance) {
        double upper_tolerance = Math.max(0, selected_speed + tolerance);
        double lower_tolerance = Math.max(0, selected_speed - tolerance);
        double currentVelocity = getCurrentVelocity();

        return currentVelocity >= lower_tolerance && currentVelocity <= upper_tolerance;
    }

    public boolean velocityDrop() {
        // Track most recent measurements
        double currentVelocity = getCurrentVelocity();
        previousShotVelocityL.add(currentVelocity);

        // Maintain window of last 50 readings
        if (previousShotVelocityL.size() > 50) {
            previousShotVelocityL.remove(0);
        }

        // Detect if significant drop occurred
        double maxRecentVelocity = previousShotVelocityL.stream()
                .mapToDouble(v -> v)
                .max()
                .orElse(currentVelocity);

        double velocityDrop = maxRecentVelocity - currentVelocity;
        return velocityDrop >= min_velocity_drop;
    }

    public void startSpeed(double selectedSpeed) {
        if (!hasStartedAutoLaunch) { // should be negated
            hasStartedAutoLaunch = true;
            decBot.flylaunch(selectedSpeed); // start up motors
        }
    }

    public void singleBallFire(double selectedSpeed, double tolerance) {
        // If speed is within range, start feeding once
        if (canLaunch(selectedSpeed, tolerance) && !hasStartedFeeding) {
            hasStartedFeeding = true;
            hasReleased = false;
            decBot.startfeeding();
        }
        boolean hasdroped = velocityDrop();
        // Stop feeding once velocity drop indicates the ball launched
        if (hasStartedFeeding && hasdroped) {
            hasReleased = true;
            hasStartedFeeding = false;
            decBot.stopfeeding();
        }
        telemetry.addData("has dropped", hasdroped);
    }

    public void launchAuto(double selectedSpeed, double tolerance) {
        startSpeed(selectedSpeed);
        int ballCount = 3;
        for (int i = 1; i <= ballCount; i++) {
            singleBallFire(selectedSpeed, tolerance);
        }
    }
    public void firingControl(){
        if (gamepad2.left_stick_button) {

        }
        if (gamepad2.x) {       // Square
            // NEAR preset
            targetVelocity = 700;
        }
        if (gamepad2.a) {   // X
            targetVelocity = 800;
        }
        if (gamepad2.b) { // Circle
            targetVelocity = 900;
        }
        if (gamepad2.y) { // Triangle
            targetVelocity = 1000;
        }

        if (gamepad2.dpad_up)     { targetVelocity += 1; }
        if (gamepad2.dpad_down)   { targetVelocity -= 1; }
        if (gamepad2.left_bumper) { targetVelocity = 0;  }

        if (gamepad2.right_bumper){
            launchAuto(targetVelocity, tolerance);
        }
    }


    public void basics(){
        speedControl();
        limeLightData();
        telemetryOutput();
        robotCentricDrive();
        LEDDriver();
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
                frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal; // Vertical + Rotation + Staffing
                frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;// Vertical - Rotation - Strafing
                rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
                rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;

                break;
            case PROFILE_2:
                //leftStickXVal controls strafing, and rightStickXVal controls rotation.
                frontLeftSpeed = leftStickYVal + rightStickXVal + leftStickXVal;    // Vertical + Rotation + Staffing
                frontRightSpeed = leftStickYVal - rightStickXVal - leftStickXVal;   // Vertical - Rotation - Strafing(sign in front is the way the motor is turning in relation to the others)
                rearLeftSpeed = leftStickYVal - rightStickXVal + leftStickXVal;
                rearRightSpeed = leftStickYVal + rightStickXVal - leftStickXVal;
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

    // ***** Helper Method for Speed Control
    public void speedControl() {
        if (gamepad1.dpad_up) {
            moveSpeedMultiply = 0.5;
        } else if (gamepad1.dpad_right) {
            moveSpeedMultiply = 0.75;
        } else if (gamepad1.dpad_down) {
            moveSpeedMultiply = 0.25;
        } else if (gamepad1.dpad_left) {
            moveSpeedMultiply = 1;
        }
    }

    //*********  Driver 2 Control Methods *****************



    // ***** Manual Feeder Wheel Controller
    /*public void feedWheelManualControl() {
        if (gamepad2.left_trigger > 0.5) {
            decBot.feedArtifact(1.0);
        }
        else if (gamepad2.right_trigger > 0.5) {
            decBot.feedArtifact(-1.0);
        }
        else if(gamepad2.left_stick_button){
            decBot.feedArtifact(0);
        }

    }*/

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

    public void autoTarget() {

    }

    // ***** Helper Method for Telemetry
    public void telemetryOutput() {
        telemetry.addData("Can fire", canLaunch(targetVelocity,tolerance));
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

    // ****** Led Controller
    public void LEDDriver() {
        decBot.LEDCon(6);
    }

     // + = tag is to the right of center, - = left.

    public Double getTagAngleDegrees(int desiredTagId) {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() == desiredTagId) {
                return fr.getTargetXDegrees();
            }
        }
        // if no limelight found
        return null;
    }

    public Double getTagDistanceMeters(int desiredTagId) {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        // yes i did steal this from andrew
        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() == desiredTagId) {
                Pose3D tagInCam = fr.getTargetPoseCameraSpace();
                double x = tagInCam.getPosition().x;
                double y = tagInCam.getPosition().y;
                double z = tagInCam.getPosition().z;

                double distanceMeters = Math.abs(Math.sqrt(x * x + y * y + z * z));
                return distanceMeters;
            }
        }
        // if no tag
        return null;
    }
    public double calculateTargetVelocity(int tag){
        double distance = getTagDistanceMeters(tag);
        double velocity = 0;
        return velocity;
    }


}