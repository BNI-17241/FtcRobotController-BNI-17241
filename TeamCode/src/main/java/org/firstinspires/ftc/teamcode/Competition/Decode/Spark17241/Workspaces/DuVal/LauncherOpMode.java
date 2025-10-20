package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.DuVal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "DuVal Launcher", group = "Lab")


public class LauncherOpMode extends OpMode {

    DcMotorEx launcherLeft = null;
    DcMotorEx launcherRight = null;
    DcMotorEx feederWheel = null;
    public double targetVelocity = 0;





    @Override
    public void init() {
        launcherLeft = hardwareMap.get(DcMotorEx.class, "left_fly_wheel");
        launcherRight = hardwareMap.get(DcMotorEx.class, "right_fly_wheel");
        feederWheel = hardwareMap.get(DcMotorEx.class,"feeder_wheel");//Port ex 2


        // Reverse one motor if needed
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);


        // Set to run using encoder
//        launcherLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        launcherRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Optional: set PIDF coefficients for velocity control (per motor)
//        PIDFCoefficients pidf = new PIDFCoefficients(50.0, 0.0, 0.0, 12.0); // Example values
//        launcherLeft.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
//        launcherRight.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);

//        targetVelocity = 2000; // Example value, tune based on motor and gear ratio

    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            targetVelocity = 1000;
            launcherLeft.setVelocity(targetVelocity);
            launcherRight.setVelocity(targetVelocity);
        }

        if (gamepad2.b) {
            launcherLeft.setPower(0);
            launcherRight.setPower(0);
        }

        if (gamepad2.right_bumper) {
            feederWheel.setPower(1);
        }
        if (gamepad2.left_bumper) {
            feederWheel.setPower(0);
        }

        if (gamepad2.dpad_up) {
            targetVelocity += 1;
        }
        if (gamepad2.dpad_down) {
            targetVelocity -= 1;
        }

//        launcherLeft.setVelocity(targetVelocity);
//        launcherRight.setVelocity(targetVelocity);

        telemetry.addData("Velocity", targetVelocity);
        telemetry.addData("MotorLeft", launcherLeft.getVelocity());
        telemetry.addData("MotorRight", launcherRight.getVelocity());
    }
}
