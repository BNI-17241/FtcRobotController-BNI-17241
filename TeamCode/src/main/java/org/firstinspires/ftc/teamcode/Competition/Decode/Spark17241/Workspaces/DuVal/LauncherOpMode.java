package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.DuVal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Disabled
@TeleOp(name = "Tester: Two Motor Velocity", group = "Lab")


public class LauncherOpMode extends OpMode {

    DcMotorEx launcherLeft = null;
    DcMotorEx launcherRight = null;
    DcMotorEx feederWheel = null;
    public double targetVelocity = 0;


    @Override
    public void init() {

        // Initiatialize Hardware
        launcherLeft = hardwareMap.get(DcMotorEx.class, "left_fly_wheel");
        launcherRight = hardwareMap.get(DcMotorEx.class, "right_fly_wheel");
        feederWheel = hardwareMap.get(DcMotorEx.class,"feeder_wheel");//Port ex 2


        // Programmatically Reverse one motor if needed
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set to run using encoder
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Optional: set PIDF coefficients for velocity control (per motor)
        PIDFCoefficients pidf = new PIDFCoefficients(90.0, 0.20, 0.20, 12.0); // Example values
        targetVelocity = 800; // Example value, tune based on motor and gear ratio

    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            targetVelocity = 1200;
        }

        if (gamepad2.b) {
            targetVelocity = 1600;
        }

        if (gamepad2.y) {
            targetVelocity = 2000;
        }

        if (gamepad2.x) {
            targetVelocity = 0;
        }


        if (gamepad2.dpad_up) {
            targetVelocity += 1;
        }
        if (gamepad2.dpad_down) {
            targetVelocity -= 1;
        }

        // USE WITH 100% Proportional Velocity, No PDIF
        launcherLeft.setVelocity(targetVelocity);
        launcherRight.setVelocity(targetVelocity);

        // USE WITH PDIF Velocity Coefficients
        // launcherLeft.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
        // launcherRight.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);

        telemetry.addData("Velocity", targetVelocity);
        telemetry.addData("MotorLeft", launcherLeft.getVelocity());
        telemetry.addData("MotorRight", launcherRight.getVelocity());
    }
}
