package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew;

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

@TeleOp(name = "AndrewAuto TeleOp", group = "Drive")
public class AndrewAimbotTeleOp17241 extends OpMode {

    private Limelight3A limelight;


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


    //Check Limelight
    public boolean limelightOn = false;


    //Auto Correct X Variation (In X values from limelight, approx +- 20 total)
    double autoVariation = 1;
    double autoOffsetFar = 3;

    //Autocorrect rotation speed
    double autoSpeed = .5;

    //Is targeting
    public boolean target = false;

    //Limelight Cam data
    public LLResult result;

    // Flywheel & Feed Wheel Variables
    public double targetVelocity = 0;

    // Drive Profile Control Variables
    public static final int PROFILE_1 = 1;  //User 1
    public static final int PROFILE_2 = 2; //user 2
    public int currentProfile = PROFILE_1;

    // Velocity gate
    double gatePercent = 0.05;            // ±5`     % gate window

    // Feed action
    double feederPower = 1.0;             // power for feeder wheel (0..1)
    long   feedMs = 700;                  // how long to run feeder

    // Shot-drop compensation (temporary target bump while feeding)
    double boostFactor = 1.02;            // +2% target during feed
    long   boostMs = 180;                 // usually ~150–250 ms

    // ===== Feeder FlyWheel Gate State Control =====
    enum ShootState { IDLE, WAIT_FOR_GATE, FEEDING, RECOVERING }
    ShootState state = ShootState.IDLE;
    ElapsedTime timer = new ElapsedTime();

    boolean rb;
    boolean rbPressed;
    boolean prevRb;


    double nominalTarget = 0;             // remembers non-boosted target
    double tolerance; // floor to 10 ticks per secibd
    boolean leftInGateStatus = false;
    boolean rightInGateStatus = false;
    boolean inGate = false;

    double currentVelocityLeft;
    double currentVelocityRight;

    // Instantiation of Robot using Robot Class Constructor
    public DecodeBot decBot = new DecodeBot();


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
        flyWheelControl();
        flyWheelStateControl();
        feedWheelManualControl();
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

        if (gamepad2.x) { targetVelocity = 876; }
        if (gamepad2.a) { targetVelocity = 934; }
        if (gamepad2.b) { targetVelocity = 1003; }
        if (gamepad2.y) { targetVelocity = 1125; }
        if (gamepad2.dpad_up) targetVelocity += 1;
        if (gamepad2.dpad_down) targetVelocity -= 1;
        if (gamepad2.left_bumper) { targetVelocity = 0; }

        decBot.flylaunch(targetVelocity);

        // Keep nominalTarget synced unless we’re in a boost
        if (state == ShootState.IDLE || state == ShootState.WAIT_FOR_GATE) {
            nominalTarget = targetVelocity;
        }

        // Always command velocity each loop
        decBot.leftFlyWheel.setVelocity(targetVelocity);
        decBot.rightFlyWheel.setVelocity(targetVelocity);

        // ===== Read velocities & gate =====
        currentVelocityLeft = decBot.leftFlyWheel.getVelocity();
        currentVelocityRight = decBot.rightFlyWheel.getVelocity();

        tolerance = Math.max(10.0, Math.abs(nominalTarget) * gatePercent); // floor to 10 ticks per secibd
        leftInGateStatus  = Math.abs(currentVelocityLeft - nominalTarget) <= tolerance;
        rightInGateStatus = Math.abs(currentVelocityRight - nominalTarget) <= tolerance;
        inGate = leftInGateStatus && rightInGateStatus;

        // ===== Shot request (rising edge on RB) =====
        rb = gamepad2.right_bumper;
        rbPressed = rb && !prevRb;
        prevRb = rb;

    }

    public void flyWheelStateControl() {
        // ===== State machine =====
        switch (state) {
            case IDLE:
                decBot.feederWheel.setPower(0);
                if (rbPressed) {
                    state = ShootState.WAIT_FOR_GATE;
                }
                break;

            case WAIT_FOR_GATE:
                decBot.feederWheel.setPower(0);
                if (inGate) {
                    // Apply brief boost and feed
                    targetVelocity = nominalTarget * boostFactor;
                    timer.reset();
                    decBot.feederWheel.setPower(feederPower);
                    state = ShootState.FEEDING;
                }
                // If driver cancels by pressing X (stop), go idle
                if (gamepad2.yWasPressed()) {
                    state = ShootState.IDLE;
                    targetVelocity = nominalTarget;
                }
                break;

            case FEEDING:
                // Maintain boost while feeding
                targetVelocity = nominalTarget * boostFactor;
                if (timer.milliseconds() >= feedMs) {
                    decBot.feederWheel.setPower(0);
                    // Start recovery (let wheel return to nominal target)
                    targetVelocity = nominalTarget;
                    timer.reset();
                    state = ShootState.RECOVERING;
                }
                break;

            case RECOVERING:
                // Give the wheel a short window to re-settle; you can also re-arm immediately.
                if (timer.milliseconds() >= boostMs) {
                    state = ShootState.IDLE;
                }
                break;
        }



    }
    // ***** Manual Feeder Wheel Controller
    public void feedWheelManualControl() {
        if (gamepad2.left_trigger > 0.5) {
            decBot.feedArtifact(1.0);
        }
        else if (gamepad2.right_trigger > 0.5) {
            decBot.feedArtifact(-1.0);
        }
        else if(gamepad2.left_stick_button){
            decBot.feedArtifact(0);
        }

    }


    //****************Limelight Data Collection
    public void limeLightData()
    {
        result = limelight.getLatestResult();
        if (result.isValid()) {
            limelightOn = true;
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();



            /*telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());

            telemetry.addData("Botpose", botpose.toString());

            // Access barcode results
            List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
            for (LLResultTypes.BarcodeResult br : barcodeResults) {
                telemetry.addData("Barcode", "Data: %s", br.getData());
            }

            // Access classifier results
            List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
            for (LLResultTypes.ClassifierResult cr : classifierResults) {
                telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
            }

            // Access detector results
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            for (LLResultTypes.DetectorResult dr : detectorResults) {
                telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
            }
            */
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }


            /*
            // Access color results
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
            }*/
        } else {
            telemetry.addData("Limelight", "No data available");
        }

    }


    //Auto Correction
    public void autoTarget()
    {

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if(fr.getFiducialId() == 20) {
                //Z pos in INCHES
                telemetry.addData("Distance from 20: ", (29.5 - 15.912402) / Math.tan(fr.getTargetYDegrees()));
                if (gamepad1.b) {
                    if (fr.getTargetXDegrees() < -autoVariation + autoOffsetFar) {
                        //Turn Left
                        setMotorPower(decBot.frontLeftMotor, -autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.frontRightMotor, autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.rearLeftMotor, -autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.rearRightMotor, autoSpeed, powerThreshold, speedMultiply);

                    }
                    if (fr.getTargetXDegrees() > autoVariation + autoOffsetFar) {
                        //Turn Right
                        setMotorPower(decBot.frontLeftMotor, autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.frontRightMotor, -autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.rearLeftMotor, autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.rearRightMotor, -autoSpeed, powerThreshold, speedMultiply);
                    }
                }
            }
            if(fr.getFiducialId() == 24)
            {
                //Z pos in INCHES
                telemetry.addData("Distance from 24: ", (29.5 - 15.912402) / Math.tan(fr.getTargetYDegrees()));
                if(gamepad1.b){
                    if(fr.getTargetXDegrees() < -autoVariation - autoOffsetFar)
                    {
                        //Turn Left
                        setMotorPower(decBot.frontLeftMotor, -autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.frontRightMotor, autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.rearLeftMotor,  -autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.rearRightMotor,  autoSpeed, powerThreshold, speedMultiply);

                    }
                    if(fr.getTargetXDegrees() > autoVariation - autoOffsetFar)
                    {
                        //Turn Right
                        setMotorPower(decBot.frontLeftMotor,  autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.frontRightMotor,-autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.rearLeftMotor,   autoSpeed, powerThreshold, speedMultiply);
                        setMotorPower(decBot.rearRightMotor, -autoSpeed, powerThreshold, speedMultiply);
                    }
                }
                //telemetry.addData("Fiducial2:", "ID: %d, X: %.2f",fr.getFiducialId(), fr.getTargetXDegrees());
                //telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        }
    }


    // ***** Helper Method for Telemetry
    public void telemetryOutput() {
        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addData("Left Fly Wheel Velocity: ", decBot.leftFlyWheel.getVelocity());
        telemetry.addData("Right Fly Wheel Velocity: ", decBot.rightFlyWheel.getVelocity());
        telemetry.addData("Launching State", state);
        telemetry.addData("Target (nominal velocity)", nominalTarget);
        telemetry.addData("Target (cmd velocity)", targetVelocity);
        telemetry.addData("Gate Tolerance ±%", gatePercent * 100.0);
        telemetry.addData("Tolerance (ticks per sec)", tolerance);
        telemetry.addData("Left Fly Wheel velocity", currentVelocityLeft);
        telemetry.addData("Right Fly Wheel velocity", currentVelocityRight);
        telemetry.addData("Left|Right inGate Status", "%b | %b", leftInGateStatus, rightInGateStatus);
        telemetry.addData("Feeder Wheel Power", decBot.feederWheel.getPower());
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
    public void LEDDriver()
    {
        if(targetVelocity == 0) {
            if(!limelightOn) {decBot.LEDCon(6);}//Purple if Limelight is not on
            else{decBot.LEDCon(5);}//Blue if 0 Velocity
        }
        else {
            if (leftInGateStatus && rightInGateStatus) {
                decBot.LEDCon(4);//Green is good
            } else {
                decBot.LEDCon(1);//Red is not ready yet
            }
        }
    }


}