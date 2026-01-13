package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot_One__Wheel_Launch;

@TeleOp(name = "Oz small Test", group = "Test")
public class test extends OpMode {

    public DecodeBot_One__Wheel_Launch decBot = new DecodeBot_One__Wheel_Launch();

    // Control variables
    protected double moveSpeedMultiply = 0.75;
    protected double targetVelocity = 0;
    protected double tolerance = 50;
    protected double powerThreshold = 0.05;

    protected double leftStickYVal, leftStickXVal, rightStickXVal;
    protected double frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed;

    // Debounce variables
    protected boolean prevLB = false, prevRB = false;
    protected boolean prevLT = false, prevRT = false;
    protected boolean prevDpadLeft = false, prevDpadRight = false;
    protected boolean prevB = false;

    @Override
    public void init() {
        decBot.initRobot(hardwareMap);
        decBot.imu.resetYaw();
    }

    @Override
    public void loop() {
        driveControl();
        intakeControl();
        launchControl();
        telemetryOutput();
        decBot.flylaunch(targetVelocity);
    }

    public void driveControl() {
        speedControl();
        robotCentricDrive();
    }

    private void robotCentricDrive() {
        leftStickYVal = -gamepad1.left_stick_y;
        leftStickXVal = gamepad1.left_stick_x;
        rightStickXVal = gamepad1.right_stick_x;

        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);

        frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
        frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
        rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
        rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;

        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);
        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

        setMotorPower(decBot.frontLeftMotor, frontLeftSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(decBot.frontRightMotor, frontRightSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(decBot.rearLeftMotor, rearLeftSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(decBot.rearRightMotor, rearRightSpeed, powerThreshold, moveSpeedMultiply);
    }

    private void speedControl() {
        if (gamepad1.dpad_up) moveSpeedMultiply = 1.0;
        else if (gamepad1.dpad_right) moveSpeedMultiply = 0.75;
        else if (gamepad1.dpad_down) moveSpeedMultiply = 0.5;
        else if (gamepad1.dpad_left) moveSpeedMultiply = 0.25;
    }

    public void intakeControl() {
        if (gamepad2.a) {
            decBot.intakeControl(false);  // Out
        } else if (gamepad2.x) {
            decBot.intakeControl(true);   // In
        }
    }

    public void launchControl() {
        // B button - Emergency stop (debounced)
        if (gamepad2.b && !prevB) {
            targetVelocity = 0;
            decBot.intakeControl(false);
        }

        // FIXED: Corrected trigger logic (debounced coarse ±100)
        if (gamepad2.left_trigger > 0.5 && !prevLT) {
            targetVelocity = Math.max(0, targetVelocity - 100);  // LT = -100
        }
        if (gamepad2.right_trigger > 0.5 && !prevRT) {
            targetVelocity += 100;  // FIXED: RT = +100 (was setting to 0)
        }

        // Bumper controls (fine ±10, debounced)
        if (gamepad2.left_bumper && !prevLB) {
            targetVelocity = Math.max(0, targetVelocity - 10);
        }
        if (gamepad2.right_bumper && !prevRB) {
            targetVelocity += 10;
        }

        // Dpad tolerance (debounced)
        if (gamepad2.dpad_left && !prevDpadLeft) {
            tolerance = Math.max(10, tolerance - 10);
        }
        if (gamepad2.dpad_right && !prevDpadRight) {
            tolerance += 10;
        }

        // Fire when ready (not debounced)
        if (gamepad2.y && canLaunch(targetVelocity, tolerance)) {
            decBot.startfeeding();
        }




        // Update debounce states
        prevB = gamepad2.b;
        prevLT = gamepad2.left_trigger > 0.5;
        prevRT = gamepad2.right_trigger > 0.5;
        prevLB = gamepad2.left_bumper;
        prevRB = gamepad2.right_bumper;
        prevDpadLeft = gamepad2.dpad_left;
        prevDpadRight = gamepad2.dpad_right;
    }

    private boolean canLaunch(double selected_speed, double tolerance) {
        if (selected_speed <= 0) return false;  // Don't fire if velocity is 0
        double currentVelocity = getCurrentVelocity();
        double lower = Math.max(0, selected_speed - tolerance);
        double upper = selected_speed + tolerance;
        return currentVelocity >= lower && currentVelocity <= upper;
    }

    private double getCurrentVelocity() {
        try {
            return (decBot.launchFrontMotor.getVelocity() + decBot.launchBackMotor.getVelocity()) / 2.0;
        } catch (Exception e) {
            return 0;
        }
    }

    public void telemetryOutput() {
        telemetry.addData("Drive Speed", String.format("%.2f", moveSpeedMultiply));
        telemetry.addData("Target Vel", String.format("%.1f", targetVelocity));
        telemetry.addData("Current Vel", String.format("%.1f", getCurrentVelocity()));
        telemetry.addData("Launch Ready", canLaunch(targetVelocity, tolerance));
        telemetry.addData("Tolerance", String.format("%.1f", tolerance));
        telemetry.addData("Intake", gamepad2.x ? "IN" : (gamepad2.a ? "OUT" : "STOP"));
        telemetry.update();
    }

    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (Math.abs(speed) <= threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }
}
