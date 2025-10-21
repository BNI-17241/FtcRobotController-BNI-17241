package org.firstinspires.ftc.teamcode.Lab;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name = "MotorSpeedControl")

public class MotorSpeedControl extends OpMode {

    //reset elepased time when press A

    DcMotor myMotor = null;


    public ElapsedTime TeleOpTime;

    public double targetHighGoalRPMs = 2500;

    public double encodersPerSecond = 0;


    public boolean initTeleOpToggle = true;
    public LinearOpMode linearOp = null;

    public double motorSpeed = 0.75;

    public double minMotorPower = 0.6;
    public double maxMotorPower = 0.9;

    public boolean powerMode = true;

    public final static int ENCODER_PPR = 28;

    public double RPMs = 0;

    public double encodersResetCount = 0;

    @Override
    public void init() {
        myMotor = hardwareMap.dcMotor.get ("launcher_motor");
        myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TeleOpTime = new ElapsedTime();
    }

    @Override
    public void loop() {
        encodersPerSecond = ((myMotor.getCurrentPosition() - encodersResetCount) / TeleOpTime.seconds());
        RPMs = (encodersPerSecond / ENCODER_PPR) * 60;

        if (initTeleOpToggle == true) {
            initTeleOp();
        }

        if (gamepad2.a) {
            resetProgram();
        }

        if (gamepad2.b) {
            normalizeEncodersManually();
        }

//        normalizeEncoders();

        if (powerMode) {
            if (gamepad2.right_bumper) {
                motorSpeed += .01;
            }
            if (gamepad2.left_bumper) {
                motorSpeed -= .01;
            }
        }

        else {
            if (gamepad2.right_bumper) {
                targetHighGoalRPMs += 10;
            }
            if (gamepad2.left_bumper) {
                targetHighGoalRPMs -= 10;
            }


            if (RPMs < targetHighGoalRPMs) {
                telemetry.addLine("ENCODERS SLOW; INCREASING MOTOR SPEED");
                motorSpeed += 0.0001;
            }

            if (RPMs > targetHighGoalRPMs) {
                telemetry.addLine("ENCODERS FAST; DECREASING MOTOR SPEED");
                motorSpeed -= 0.0001;
            }
        }

        motorSpeed = Range.clip(motorSpeed, minMotorPower, maxMotorPower);
        myMotor.setPower(motorSpeed);


        if (gamepad2.dpad_right) {
            powerMode = true;
        }

        if (gamepad2.dpad_left) {
            powerMode = false;
        }

        displayTelemetry();

    }

    public void setMotorRunModes (DcMotor.RunMode mode) {
        myMotor.setMode(mode);
    }

    public void setLinearOp(LinearOpMode linearOp) {

        this.linearOp = linearOp;
    }

    public void displayTelemetry () {
        telemetry.addLine("Press A to reset timer and encoders");
        telemetry.addLine("Dpad R:control power motor; Dpad L: Control encoders");
        if (powerMode == true) {
            telemetry.addLine("MOTOR POWER MODE");
            telemetry.addLine("RB: Increase Motor Speed");
            telemetry.addLine("LB: Decrease Motor Speed");
        }

        if (powerMode == false) {
            telemetry.addLine("ENCODER CONTROL MODE");
            telemetry.addLine("RB: Increase Encoders per Second");
            telemetry.addLine("LB: Decrease Encoders per Second");
            telemetry.addData("TARGET ENCODERS PER SECOND: ", targetHighGoalRPMs);
        }

        telemetry.addData("Current Motor Power: ", myMotor.getPower());
        telemetry.addData("Current Time (ms): ", TeleOpTime.seconds());
//        telemetry.addData("Time since last reset", TeleOpTime.seconds());
        telemetry.addData("Current Encoders: ", myMotor.getCurrentPosition());
        telemetry.addData("Encoders per second: ", encodersPerSecond);
//        telemetry.addData("Encoders per 1 minute: ", (encodersPerSecond * 60));
        telemetry.addData("RPMs!: ", RPMs);
    }

    public void resetProgram () {
        TeleOpTime.reset();
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encodersResetCount = 0;
    }

    public void normalizeEncodersManually () {
        TeleOpTime.reset();
        encodersResetCount = encodersResetCount + myMotor.getCurrentPosition();
    }

    public void normalizeEncoders () {
        if (TeleOpTime.seconds() >= 3) {
            TeleOpTime.reset();
            encodersResetCount = encodersResetCount + myMotor.getCurrentPosition();
        }
    }

    public void initTeleOp () {
        TeleOpTime.reset();
        initTeleOpToggle = false;               // false so initializes only once
    }
}
