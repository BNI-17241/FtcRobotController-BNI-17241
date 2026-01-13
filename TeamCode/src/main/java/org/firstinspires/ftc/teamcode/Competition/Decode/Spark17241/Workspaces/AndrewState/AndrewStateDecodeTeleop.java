package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.AndrewState;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;

import java.util.List;

@TeleOp(name = "Andrew State Decode Teleop", group = "Drive")
public class AndrewStateDecodeTeleop extends OpMode {

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
    protected double speedMultiply = 0.75;

    // Drive Profile Control Variables
    protected  static final int PROFILE_1 = 1;  //User 1
    protected  static final int PROFILE_2 = 2; //user 2
    protected  int currentProfile = PROFILE_1;

    //Limelight Cam data
    protected  LLResult result;


    //Velocity of the Launching wheel
    protected double currentLaunchVelocity;


    // Instantiation of Robot using Robot Class Constructor
    public AndrewStateDecodeBot decBot = new AndrewStateDecodeBot();


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
        speedControl();
        limeLightData();
        autoTarget();
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
        if (result == null || !result.isValid()) {
            telemetry.addData("AutoTarget", "No valid Limelight result");
            return;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        telemetry.addData("Result", fiducialResults);
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            Pose3D tagInCam = fr.getTargetPoseCameraSpace();     // pose of tag in CAMERA space
            double x = tagInCam.getPosition().x;  // right (+)
            double y = tagInCam.getPosition().y;  // down (+)
            double forwardMeters = tagInCam.getPosition().z;  // forward/out of camera (+)

            double rangeMeters = Math.sqrt(x * x + y * y + forwardMeters * forwardMeters);// 3D range

            telemetry.addLine("-------------------------------------");
            telemetry.addData("Tag ID", fr.getFiducialId());
            telemetry.addData("Forward (m)", "%.3f", forwardMeters);
            telemetry.addData("Range (m)", "%.3f", rangeMeters);
            telemetry.addData("Yaw", fr.getTargetXDegrees());
            telemetry.addLine("-------------------------------------");
        }
    }

    // ***** Helper Method for Telemetry
    public void telemetryOutput() {
        telemetry.addLine("-------------------------------------");
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
}