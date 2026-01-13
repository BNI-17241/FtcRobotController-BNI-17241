package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot_One__Wheel_Launch;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Oz Test", group = "Drive")
public class test extends OpMode {

    protected Limelight3A limelight;

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

    protected static final int PROFILE_1 = 1;
    protected static final int PROFILE_2 = 2;
    protected int currentProfile = PROFILE_1;

    protected LLResult result;

    protected double targetVelocity = 0;
    protected double tolerance = 50;

    protected double min_velocity_drop = 50;
    protected List<Double> previousShotVelocityL = new ArrayList<>();
    protected boolean hasStartedAutoLaunch = false;
    protected boolean hasStartedFeeding = false;
    protected boolean hasReleased = true;
    public DecodeBot_One__Wheel_Launch decBot = new DecodeBot_One__Wheel_Launch();

    @Override
    public void init() {
        decBot.initRobot(hardwareMap);
        decBot.imu.resetYaw();
    }

    @Override
    public void loop() {
        firingControl();
        driveControl();
        telemetryOutput();
    }

    private double getCurrentVelocity() {
        return (decBot.launchFrontMotor.getVelocity() + decBot.launchBackMotor.getVelocity()) / 2.0;
    }

    public boolean canLaunch(double selected_speed, double tolerance) {
        double upper_tolerance = Math.max(0, selected_speed + tolerance);
        double lower_tolerance = Math.max(0, selected_speed - tolerance);
        double currentVelocity = getCurrentVelocity();
        return currentVelocity >= lower_tolerance && currentVelocity <= upper_tolerance;
    }

    public boolean velocityDrop() {
        double currentVelocity = getCurrentVelocity();
        previousShotVelocityL.add(currentVelocity);

        if (previousShotVelocityL.size() > 50) {
            previousShotVelocityL.remove(0);
        }

        double maxRecentVelocity = previousShotVelocityL.stream()
                .mapToDouble(v -> v)
                .max()
                .orElse(currentVelocity);

        double velocityDrop = maxRecentVelocity - currentVelocity;
        return velocityDrop >= min_velocity_drop;
    }

    public void startSpeed(double selectedSpeed) {
        if (!hasStartedAutoLaunch) {
            hasStartedAutoLaunch = true;
            decBot.flylaunch(selectedSpeed);
        }
    }

    public void singleBallFire(double selectedSpeed, double tolerance) {
        if (canLaunch(selectedSpeed, tolerance) && !hasStartedFeeding) {
            hasStartedFeeding = true;
            hasReleased = false;
            decBot.startfeeding();
        }

        boolean hasDropped = velocityDrop();

        if (hasStartedFeeding && hasDropped) {
            hasReleased = true;
            hasStartedFeeding = false;
            decBot.stopfeeding();
        }
    }

    public void launchAuto(double selectedSpeed, double tolerance) {
        startSpeed(selectedSpeed);
        // this loop as written will execute in a single loop() call and
        // rely on velocityDrop() over time; consider state machine if needed
        for (int i = 0; i < 3; i++) {
            singleBallFire(selectedSpeed, tolerance);
        }
    }

    public void firingControl() {

        if (gamepad2.x) {
            targetVelocity = 700;
        }
        if (gamepad2.a) {
            targetVelocity = 800;
        }
        if (gamepad2.b) {
            targetVelocity = 900;
        }
        if (gamepad2.y) {
            targetVelocity = 1000;
        }

        // Velocity adjustments:
        // LT -100, RT +100, LB -50, RB +50
        if (gamepad2.left_trigger > 0.5) {
            targetVelocity -= 100;
        }
        if (gamepad2.right_trigger > 0.5) {
            targetVelocity += 100;
        }
        if (gamepad2.left_bumper) {
            targetVelocity -= 50;
        }
        if (gamepad2.right_bumper) {
            targetVelocity += 50;
        }

        // Optional tolerance control on gamepad2 dpad
        if (gamepad2.dpad_left) {
            tolerance = Math.max(0, tolerance - 10);
        }
        if (gamepad2.dpad_right) {
            tolerance += 10;
        }

        // Fire sequence button (example: gamepad2.start)
        if (gamepad2.start) {
            launchAuto(targetVelocity, tolerance);
        }
    }

    // ======== Movement (gamepad1) ========

    public void driveControl() {
        speedControl();
        robotCentricDrive();
    }

    public void robotCentricDrive() {
        leftStickYVal = -gamepad1.left_stick_y;
        leftStickYVal = Range.clip(leftStickYVal, -1, 1);

        rightStickYVal = gamepad1.right_stick_y;
        rightStickYVal = Range.clip(rightStickYVal, -1, 1);

        leftStickXVal = gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);

        rightStickXVal = gamepad1.right_stick_x;
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);

        switch (currentProfile) {
            case PROFILE_1:
                frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
                frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
                rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
                rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
                break;

            case PROFILE_2:
                frontLeftSpeed = leftStickYVal + rightStickXVal + leftStickXVal;
                frontRightSpeed = leftStickYVal - rightStickXVal - leftStickXVal;
                rearLeftSpeed = leftStickYVal - rightStickXVal + leftStickXVal;
                rearRightSpeed = leftStickYVal + rightStickXVal - leftStickXVal;
                break;

            default:
                frontLeftSpeed = 0;
                frontRightSpeed = 0;
                rearLeftSpeed = 0;
                rearRightSpeed = 0;
                break;
        }

        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);
        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

        setMotorPower(decBot.frontLeftMotor, frontLeftSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(decBot.frontRightMotor, frontRightSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(decBot.rearLeftMotor, rearLeftSpeed, powerThreshold, moveSpeedMultiply);
        setMotorPower(decBot.rearRightMotor, rearRightSpeed, powerThreshold, moveSpeedMultiply);
    }

    public void speedControl() {
        if (gamepad1.dpad_up) {
            moveSpeedMultiply = 0.5;
        } else if (gamepad1.dpad_right) {
            moveSpeedMultiply = 0.75;
        } else if (gamepad1.dpad_down) {
            moveSpeedMultiply = 0.25;
        } else if (gamepad1.dpad_left) {
            moveSpeedMultiply = 1.0;
        }
    }

    // ======== Telemetry ========

    public void telemetryOutput() {
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", getCurrentVelocity());
        telemetry.addData("In Gate", canLaunch(targetVelocity, tolerance));
        telemetry.addData("Thinks Fired", hasReleased || velocityDrop());
        telemetry.addData("Tolerance", tolerance);
        telemetry.update();
    }

    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }
}
