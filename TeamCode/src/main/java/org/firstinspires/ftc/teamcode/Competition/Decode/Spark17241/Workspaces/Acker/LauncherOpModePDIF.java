package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Acker;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "Tester: Velocity with Gate", group = "Lab")


public class LauncherOpModePDIF extends OpMode {

    DcMotorEx launcherLeft = null;
    DcMotorEx launcherRight = null;
    DcMotorEx feederWheel = null;
    public double targetVelocity = 0;

    // Optional: set PIDF coefficients for velocity control (per motor)
    double kP = 0.18;
    double kI = 0.0;
    double kD = 0.01;
    double kF = 5.0;

    // Velocity gate
    double gatePercent = 0.02;            // ±2% gate window

    // Feed action
    double feederPower = 1.0;             // power for feeder wheel (0..1)
    long   feedMs = 700;                  // how long to run feeder

    // Shot-drop compensation (temporary target bump while feeding)
    double boostFactor = 1.02;            // +2% target during feed
    long   boostMs = 180;                 // usually ~150–250 ms

    boolean rb = gamepad2.right_bumper;
    boolean rbPressed = false;
    boolean prevRb = false;

    // ===== Internal =====
    enum ShootState { IDLE, WAIT_FOR_GATE, FEEDING, RECOVERING }
    ShootState state = ShootState.IDLE;


    ElapsedTime   timer = new ElapsedTime();
    double nominalTarget = 0;             // remembers non-boosted target

    @Override
    public void init() {

        // Initiatialize Hardware
        launcherLeft = hardwareMap.get(DcMotorEx.class, "left_fly_wheel");
        launcherRight = hardwareMap.get(DcMotorEx.class, "right_fly_wheel");
        feederWheel = hardwareMap.get(DcMotorEx.class,"feeder_wheel");//Port ex 2


        // Programmatically Reverse one motor if needed
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);
        feederWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set to run using encoder
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feederWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feederWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void loop() {
        // ===== Target presets =====
        if (gamepad2.a) targetVelocity = 1000;
        if (gamepad2.b) targetVelocity = 1100;
        if (gamepad2.y) targetVelocity = 1254;
        if (gamepad2.x) targetVelocity = 0;

        // Fine adjust
        if (gamepad2.dpad_up) targetVelocity += 1;
        if (gamepad2.dpad_down) targetVelocity -= 1;

        // Keep nominalTarget synced unless we’re in a boost
        if (state == ShootState.IDLE || state == ShootState.WAIT_FOR_GATE) {
            nominalTarget = targetVelocity;
        }

        // Always command velocity each loop
        launcherLeft.setVelocity(targetVelocity);
        launcherRight.setVelocity(targetVelocity);

        // ===== Read velocities & gate =====
        double currentVelocityLeft = launcherLeft.getVelocity();
        double currentVelocityRight = launcherRight.getVelocity();

        double tolerance = Math.max(10.0, Math.abs(nominalTarget) * gatePercent); // floor to 10 ticks per secibd
        boolean leftInGateStatus  = Math.abs(currentVelocityLeft - nominalTarget) <= tolerance;
        boolean rightInGateStatus = Math.abs(currentVelocityRight - nominalTarget) <= tolerance;
        boolean inGate = leftInGateStatus && rightInGateStatus;

        // ===== Shot request (rising edge on RB) =====
        rb = gamepad2.right_bumper;
        rbPressed = rb && !prevRb;
        prevRb = rb;

        // ===== State machine =====
        switch (state) {
            case IDLE:
                feederWheel.setPower(0);
                if (rbPressed) {
                    state = ShootState.WAIT_FOR_GATE;
                }
                break;

            case WAIT_FOR_GATE:
                feederWheel.setPower(0);
                if (inGate) {
                    // Apply brief boost and feed
                    targetVelocity = nominalTarget * boostFactor;
                    timer.reset();
                    feederWheel.setPower(feederPower);
                    state = ShootState.FEEDING;
                }
                // If driver cancels by pressing X (stop), go idle
                if (gamepad2.x) {
                    state = ShootState.IDLE;
                    targetVelocity = nominalTarget;
                }
                break;

            case FEEDING:
                // Maintain boost while feeding
                targetVelocity = nominalTarget * boostFactor;
                if (timer.milliseconds() >= feedMs) {
                    feederWheel.setPower(0);
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

        // ===== Telemetry =====
        telemetry.addData("Launching State", state);
        telemetry.addData("Target (nominal velocity)", nominalTarget);
        telemetry.addData("Target (cmd velocity)", targetVelocity);
        telemetry.addData("Gate Tolerance ±%", gatePercent * 100.0);
        telemetry.addData("Tolerance (ticks per sec)", tolerance);
        telemetry.addData("Left Fly Wheel velocity", currentVelocityLeft);
        telemetry.addData("Right Fly Wheel velocity", currentVelocityRight);
        telemetry.addData("Left|Right inGate Status", "%b | %b", leftInGateStatus, rightInGateStatus);
        telemetry.addData("Feeder Wheel Power", feederWheel.getPower());
        telemetry.update();
    }
}
