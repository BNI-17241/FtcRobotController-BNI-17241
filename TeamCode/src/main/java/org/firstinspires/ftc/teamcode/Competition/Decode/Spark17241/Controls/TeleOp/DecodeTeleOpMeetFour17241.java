package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;

import java.util.List;
@Disabled
@TeleOp(name = "DecodeTeleOpMeetFour17241", group = "Drive")
public class DecodeTeleOpMeetFour17241 extends OpMode {

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


    //Check Limelight
    protected  boolean limelightOn = false;


    //Auto Correct X Variation (In X values from limelight, approx +- 20 total)
    protected double autoVariation = 1;
    protected double autoOffsetFar = 2.8;

    //Autocorrect rotation speed
    protected double autoSpeed = .5;

    //Is targeting
    protected  boolean target = false;

    //Limelight Cam data
    protected  LLResult result;

    // Flywheel & Feed Wheel Variables
    protected  double targetVelocity = 0;

    // Drive Profile Control Variables
    protected  static final int PROFILE_1 = 1;  //User 1
    protected  static final int PROFILE_2 = 2; //user 2
    protected  int currentProfile = PROFILE_1;

    // Velocity Gates & Recovery Times for Near vs Far
    protected enum RangeMode { NEAR, FAR }
    protected RangeMode rangeMode = RangeMode.NEAR;
    protected double nearGatePercent = 0.10;      // ±10% window for NEAR shots
    protected double farGatePercent  = 0.03;      // ±3% window for FAR shots (your old value)
    protected double nearRecoverMs = 60;            // Near Gate Recovery MS between shots
    protected double farRecoverMs  = 180;           // Far Gate Recovery MS between shots


    // Feed action
    protected double feederPower = 1.0;             // power for feeder wheel (0..1)
    protected double   feedMs = 350;                  // how long to run feeder was 700

    // Shot-drop compensation (temporary target bump while feeding)
    protected double boostFactor = 1.02;            // +2% target during feed

    // ===== Feeder FlyWheel Gate State Control =====
    protected enum ShootState { IDLE, WAIT_FOR_GATE, FEEDING, RECOVERING }
    protected ShootState state = ShootState.IDLE;
    protected ElapsedTime timer = new ElapsedTime();

    protected boolean rb;
    protected boolean rbPressed;
    protected boolean prevRb;

    protected  boolean autoFire;
    protected  double autoTargetSpeed = 0.0;
    protected double angleTopTri;

    protected double nominalTarget = 0;             // remembers non-boosted target
    protected double tolerance; // floor to 10 ticks per secibd
    protected boolean leftInGateStatus = false;
    protected  boolean rightInGateStatus = false;
    protected  boolean inGate = false;

    protected double currentVelocityLeft;
    protected double currentVelocityRight;

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

        if (gamepad2.left_stick_button) {
            targetVelocity = -600;
            rangeMode = RangeMode.NEAR;
        }
        if (gamepad2.x) {       // Square
            // NEAR preset
            targetVelocity = 705;
            autoOffsetFar = 0;
            rangeMode = RangeMode.NEAR;
        }
        if (gamepad2.a) {   // X
            // NEAR preset
            targetVelocity = 725;  //781
            autoOffsetFar = 0;
            rangeMode = RangeMode.NEAR;
        }
        if (gamepad2.b) { // Circle
            // FAR preset
            targetVelocity = 843;
            autoOffsetFar = 2.5;
            rangeMode = RangeMode.FAR;
        }
        if (gamepad2.y) { // Triangle
            // FAR preset
            targetVelocity = 865;
            autoOffsetFar = 1.9;
            rangeMode = RangeMode.FAR;
        }

        if (gamepad2.dpad_up)     { targetVelocity += 1; }
        if (gamepad2.dpad_down)   { targetVelocity -= 1; }
        if (gamepad2.left_bumper) { targetVelocity = 0;  }

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

        double gatePercentUsed = (rangeMode == RangeMode.FAR) ? farGatePercent : nearGatePercent;
        tolerance = Math.max(10.0, Math.abs(nominalTarget) * gatePercentUsed);

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
                if (rbPressed || autoFire) {
                    timer.reset();
                    state = ShootState.WAIT_FOR_GATE;
                }
                break;

            case WAIT_FOR_GATE:
                decBot.feederWheel.setPower(0);

                if (autoFire) {
                    nominalTarget = autoTargetSpeed;
                }

                // Decide readiness based on range
                boolean gateReady;

                if (rangeMode == RangeMode.FAR) {
                    // FAR: require tight inGate on BOTH wheels
                    gateReady = inGate;
                } else {
                    // NEAR: allow looser conditions:
                    // - average velocity close enough OR
                    // - we’ve been spinning for at least 150 ms
                    double avgVel = 0.5 * (currentVelocityLeft + currentVelocityRight);
                    double gatePercentUsed = farGatePercent; // or nearGatePercent; either is fine since
                    // your near tolerance is already looser above
                    double localTol = Math.max(10.0, Math.abs(nominalTarget) * gatePercentUsed);

                    boolean avgReady  = Math.abs(avgVel - nominalTarget) <= localTol;
                    boolean timeReady = timer.milliseconds() >= 150;   // small spin-up for near shots

                    gateReady = avgReady || timeReady;
                }

                if (gateReady) {
                    // Apply brief boost and feed
                    targetVelocity = nominalTarget * boostFactor;
                    timer.reset();
                    decBot.feederWheel.setPower(feederPower);
                    state = ShootState.FEEDING;
                }

                // If driver cancels by pressing Y, go idle
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
                    autoFire = false;
                }
                break;

            case RECOVERING:
                double recoverMs = (rangeMode == RangeMode.FAR) ? farRecoverMs : nearRecoverMs;
                if (timer.milliseconds() >= recoverMs) {
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


            /*if (!result.getFiducialResults().isEmpty()) {
                List<LLResultTypes.FiducialResult> allTags = result.getFiducialResults();   // best tag this frame
                LLResultTypes.FiducialResult f = allTags.get(0);
                for(int i = 0; i < allTags.size(); i++) {
                    if(allTags.get(i).getFiducialId() == 20 || allTags.get(i).getFiducialId() == 24) {
                        f = allTags.get(i);
                    }
                }

                Pose3D tagInCam = f.getTargetPoseCameraSpace();     // pose of tag in CAMERA space
                double x = tagInCam.getPosition().x;  // right (+)
                double y = tagInCam.getPosition().y;  // down (+)
                double forwardMeters = tagInCam.getPosition().z;  // forward/out of camera (+)

                double rangeMeters = Math.sqrt(x * x + y * y + forwardMeters * forwardMeters);// 3D range

                telemetry.addLine("-------------------------------------");

                telemetry.addData("Tag ID", f.getFiducialId());
                telemetry.addData("Forward (m)", "%.3f", forwardMeters);
                telemetry.addData("Range (m)", "%.3f", rangeMeters);

            } else {
                telemetry.addData("Tag", "none");
            }

            telemetry.addLine("-------------------------------------");*/


            if(fr.getFiducialId() == 20) {
                Pose3D tagInCam = fr.getTargetPoseCameraSpace();     // pose of tag in CAMERA space
                double x = tagInCam.getPosition().x;  // right (+)
                double y = tagInCam.getPosition().y;  // down (+)
                double forwardMeters = tagInCam.getPosition().z;  // forward/out of camera (+)

                double rangeMeters = Math.sqrt(x * x + y * y + forwardMeters * forwardMeters);// 3D range

                //autoTargetSpeed = (127.7 * Math.pow(forwardMeters, 5)) + (-641.4 * Math.pow(forwardMeters, 4)) + (782.4 * Math.pow(forwardMeters, 3)) + (604.8 * Math.pow(forwardMeters, 2) - (1500 * forwardMeters) + 1500);

                telemetry.addLine("-------------------------------------");
                telemetry.addData("Tag ID", fr.getFiducialId());
                telemetry.addData("Forward (m)", "%.3f", forwardMeters);
                telemetry.addData("Range (m)", "%.3f", rangeMeters);
                telemetry.addData("Yaw", fr.getTargetXDegrees());
                telemetry.addLine("-------------------------------------");



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
                Pose3D tagInCam = fr.getTargetPoseCameraSpace();     // pose of tag in CAMERA space
                double x = tagInCam.getPosition().x;  // right (+)
                double y = tagInCam.getPosition().y;  // down (+)
                double forwardMeters = tagInCam.getPosition().z;  // forward/out of camera (+)
                double yaw = fr.getTargetXDegrees();

                double rangeMeters = Math.sqrt(x * x + y * y + forwardMeters * forwardMeters);// 3D range

                //autoTargetSpeed = (127.7 * Math.pow(forwardMeters, 5)) + (-641.4 * Math.pow(forwardMeters, 4)) + (782.4 * Math.pow(forwardMeters, 3)) + (604.8 * Math.pow(forwardMeters, 2) - (1500 * forwardMeters) + 1500);

                //angleTopTri = Math.acos((forwardMeters + 18.5 * Math.sin(yaw))/(Math.sqrt(Math.pow(forwardMeters, 2) * Math.pow(Math.cos(yaw), 2) + Math.pow(18.5 + forwardMeters * Math.sin(yaw), 2)   )));

                telemetry.addData("Angle to point A: ", angleTopTri);

                telemetry.addLine("-------------------------------------");
                telemetry.addData("Tag ID", fr.getFiducialId());
                telemetry.addData("Forward (m)", "%.3f", forwardMeters);
                telemetry.addData("Range (m)", "%.3f", rangeMeters);
                telemetry.addLine("-------------------------------------");

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
        double currentGatePercent = (rangeMode == RangeMode.FAR) ? farGatePercent : nearGatePercent;
        telemetry.addData("Gate Tolerance ±%", currentGatePercent * 100.0);
        telemetry.addData("Tolerance (ticks per sec)", tolerance);
        telemetry.addData("Feeder Wheel Power", decBot.feederWheel.getPower());
        boolean ledReady = isShotReadyForLED();
        telemetry.addData("Range Mode", rangeMode);
        telemetry.addData("LED Ready", ledReady);
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

    // Returns whether we consider the shooter "ready" for LED purposes
    public boolean isShotReadyForLED() {
        // No target = not ready
        if (targetVelocity == 0) {
            return false;
        }

        if (rangeMode == RangeMode.FAR) {
            // FAR: use strict gate on BOTH flywheels
            return inGate;
        } else {
            // NEAR: use average velocity with a looser window
            double avgVel = 0.5 * (currentVelocityLeft + currentVelocityRight);

            // Use the near gate percent here
            double ledTol = Math.max(10.0, Math.abs(nominalTarget) * nearGatePercent);

            return Math.abs(avgVel - nominalTarget) <= ledTol;
        }
    }

    // ****** Led Controller
    public void LEDDriver()
    {
        if (targetVelocity == 0) {
            // Shooter off
            if (!limelightOn) {
                decBot.LEDCon(6);  // Purple if Limelight is not on
            } else {
                decBot.LEDCon(5);  // Blue if 0 Velocity but Limelight on
            }
        } else {
            // Shooter spinning: use our "ready" helper
            boolean ledReady = isShotReadyForLED();

            if (ledReady) {
                decBot.LEDCon(4);  // Green is good (ready to shoot)
            } else {
                decBot.LEDCon(1);  // Red is not ready yet
            }
        }
    }


}