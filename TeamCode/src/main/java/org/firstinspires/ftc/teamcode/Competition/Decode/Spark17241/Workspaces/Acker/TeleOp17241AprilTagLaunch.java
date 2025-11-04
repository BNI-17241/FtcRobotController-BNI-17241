package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Acker;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name = "Tester: April Tag Launch", group = "Lab")
public class TeleOp17241AprilTagLaunch extends OpMode {

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

    // Drive Profile Control Variables
    public static final int PROFILE_1 = 1;  //User 1
    public static final int PROFILE_2 = 2; //user 2
    public int currentProfile = PROFILE_1;

    // Velocity gate
    double gatePercent = 0.05;            // ±5% gate window

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
    double tolerance;                     // floor to 10 ticks per second
    boolean leftInGateStatus = false;
    boolean rightInGateStatus = false;
    boolean inGate = false;

    double currentVelocityLeft;
    double currentVelocityRight;

    // ====== Limelight & Auto-Align Assist  ======

    // Driver holds LB (gamepad1) to engage a brief auto-align process.
    private boolean autoAlignActive = false;
    private boolean autoAligned = false;
    private ElapsedTime autoAlignTimer = new ElapsedTime();

    // Which tags matter for launching
    private static final int BLUE_TAG = 20, RED_TAG = 24;

    // Vision targets (tune to your camera mount & shot)
    private double desiredTxDeg = 0.0;          // want tag centered sideways
    private double desiredTa    = 3.0;          // target area == desired distance

    // Tolerances (tune)
    private double txTolDeg = 1.5;              // ±1.5° counts as centered
    private double taTol    = 0.35;             // ±0.35 area units counts as in range

    // Gains (gentle)
    private double kP_strafe = 0.025;           // power per degree of tx
    private double kP_drive  = 0.08;            // power per (desiredTa - ta)

    // Limits & safety
    private double alignMaxStrafe = 0.35;       // max strafe assist
    private double alignMaxDrive  = 0.30;       // max forward/back assist
    private double stickCancel    = 0.15;       // driver stick bump cancels assist
    private double deadbandTx     = 0.30;       // deg
    private double deadbandTa     = 0.08;       // area
    private double alignTimeoutS  = 1.75;       // seconds

    // LED codes
    private static final int LED_AIMING = 2;
    private static final int LED_LOCKED = 3;


    // Instantiation of Robot using Robot Class Constructor
    public DecodeBot_Acker decBot = new DecodeBot_Acker();


    @Override
    public void init() {
        decBot.initRobot(hardwareMap);
        decBot.imu.resetYaw(); // REV
        decBot.initLimelight(hardwareMap);
    }

    @Override
    public void loop() {
        speedControl();

        // ----- AUTO-ALIGN ASSIST: takes over only while LB is held and sticks are quiet -----
        boolean assisted = runAutoAlignAssist();

        if (!assisted) {
            // Normal manual drive if assist is not active
            robotCentricDrive();
        }

        // Launcher system
        flyWheelControl();
        flyWheelStateControl();
        feedWheelManualControl();

        // LED Controller
        LEDDriver();

        // If we are aiming or locked, override LED to show status clearly
        if (autoAlignActive && !autoAligned) {
            decBot.LEDCon(LED_AIMING);
        }
        if (autoAligned && inGate) {
            decBot.LEDCon(LED_LOCKED); // “ready to shoot” indicator
        }

        // Limelight controls & stream debug
        limelightStatusControl();
        limelightStreamResults();

        // Flywheel & general telemetry
        telemetryOutput();
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
                frontLeftSpeed = leftStickYVal + rightStickXVal + leftStickXVal;    // Vertical + Rotation + Strafing
                frontRightSpeed = leftStickYVal - rightStickXVal - leftStickXVal;   // Vertical - Rotation - Strafing
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

            default: // stops robot
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

        tolerance = Math.max(10.0, Math.abs(nominalTarget) * gatePercent); // floor to 10 tps
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
                }
                break;

            case RECOVERING:
                // Give the wheel a short window to re-settle
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
        telemetry.addData("AutoAlign Active|Locked", "%b | %b", autoAlignActive, autoAligned);
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
        if(targetVelocity == 0){
            decBot.LEDCon(5);
        } else {
            if (leftInGateStatus && rightInGateStatus) {
                decBot.LEDCon(4);
            } else {
                decBot.LEDCon(1);
            }
        }
    }

    // ***** Limelight start/stop (FIXED: up=start, down=stop)
    public void limelightStatusControl() {
        if (gamepad2.dpad_up) {
            decBot.startLimelight();
        }
        if (gamepad2.dpad_down) {
            decBot.stopLimelight();
        }
    }

    public void limelightStreamResults(){
        LLResult result = decBot.limelight.getLatestResult();
        telemetry.addData("LL Result: ", result);

        //Fiducial results is a list - if exactly one tag, show which ID
        if (result != null && result.getFiducialResults() != null && result.getFiducialResults().size() == 1){
            telemetry.addData("Tags", result.getFiducialResults().get(0).getFiducialId());
        }
    }

    // ===== Auto-Align helpers (NEW) =====

    /** Return the latest LLResult if it includes at least one of our target tags; else null. */
    public LLResult getLatestDesiredTagResult() {
        LLResult r = decBot.limelight.getLatestResult();
        if (r == null || r.getFiducialResults() == null || r.getFiducialResults().isEmpty()) return null;

        // Prefer one of our tags (20 or 24)
        int n = r.getFiducialResults().size();
        for (int i = 0; i < n; i++) {
            int id = r.getFiducialResults().get(i).getFiducialId();
            if (id == BLUE_TAG || id == RED_TAG) return r;
        }
        return null;
    }

    /** True if we’re within tolerance in both axes. */
    public boolean isAligned(double txDeg, double ta) {
        return Math.abs(txDeg - desiredTxDeg) <= txTolDeg &&
                Math.abs(ta - desiredTa)       <= taTol;
    }

    /**
     * While LB held and sticks quiet, gently strafe from tx and drive from ta until aligned or timeout.
     * Returns true if this method commanded the drivetrain (i.e., assist took control this loop).
     */
    public boolean runAutoAlignAssist() {
        boolean wantAssist = gamepad1.left_bumper;

        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y; // forward is +
        double rx = gamepad1.right_stick_x;

        // If driver moves sticks, cancel
        if (Math.abs(lx) > stickCancel || Math.abs(ly) > stickCancel || Math.abs(rx) > stickCancel) {
            autoAlignActive = false;
            autoAligned = false;
            return false;
        }

        if (!wantAssist) {
            autoAlignActive = false;
            autoAligned = false;
            return false;
        }

        // Need a valid tag
        LLResult r = getLatestDesiredTagResult();
        if (r == null) {
            autoAlignActive = false;
            autoAligned = false;
            return false;
        }

        autoAlignActive = true;

        double tx = r.getTx();  // horizontal offset (deg)
        double ta = r.getTa();  // target area (proxy for distance)

        // Deadbands
        double txErr = tx - desiredTxDeg;
        if (Math.abs(txErr) < deadbandTx) txErr = 0.0;

        double taErr = desiredTa - ta;    // positive => move forward
        if (Math.abs(taErr) < deadbandTa) taErr = 0.0;

        // Convert errors to robot-centric commands (strafe from tx, forward from area)
        double strafeCmd = Range.clip(kP_strafe * txErr, -alignMaxStrafe, alignMaxStrafe);
        double driveCmd  = Range.clip(kP_drive  * taErr, -alignMaxDrive,  alignMaxDrive);
        double turnCmd   = 0.0; // we center by strafing, not turning. Change if you prefer turn-to-target.

        // Stop if aligned or timeout
        if (autoAlignTimer.seconds() == 0) autoAlignTimer.reset();
        boolean timedOut = autoAlignTimer.seconds() > alignTimeoutS;
        autoAligned = isAligned(tx, ta) || (!isAligned(tx, ta) && timedOut);

        if (autoAligned) {
            strafeCmd = 0; driveCmd = 0; turnCmd = 0;
        }

        // Drive like your robot-centric mixer
        double fl = driveCmd + turnCmd + strafeCmd;
        double fr = driveCmd - turnCmd - strafeCmd;
        double rl = driveCmd - turnCmd + strafeCmd;
        double rr = driveCmd + turnCmd - strafeCmd;

        // Clip & apply speedMultiply; threshold=0 so assist actually drives
        setMotorPower(decBot.frontLeftMotor,  Range.clip(fl, -1, 1), 0, speedMultiply);
        setMotorPower(decBot.frontRightMotor, Range.clip(fr, -1, 1), 0, speedMultiply);
        setMotorPower(decBot.rearLeftMotor,   Range.clip(rl, -1, 1), 0, speedMultiply);
        setMotorPower(decBot.rearRightMotor,  Range.clip(rr, -1, 1), 0, speedMultiply);

        // Add concise telemetry; full update occurs in telemetryOutput()
        telemetry.addLine("=== Auto Align ===");
        telemetry.addData("tx(deg)", "%.2f", r.getTx());
        telemetry.addData("ta",      "%.2f", r.getTa());
        telemetry.addData("strafe",  "%.2f", strafeCmd);
        telemetry.addData("drive",   "%.2f", driveCmd);
        telemetry.addData("aligned", autoAligned);

        return true; // we drove for the loop
    }


}