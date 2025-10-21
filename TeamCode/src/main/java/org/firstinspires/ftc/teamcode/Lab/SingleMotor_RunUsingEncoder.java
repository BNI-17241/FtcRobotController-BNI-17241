package org.firstinspires.ftc.teamcode.Lab;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Single Motor Velocity Tester", group="Lab")

public class SingleMotor_RunUsingEncoder extends OpMode {
    private DcMotorEx motor_flywheel = null;

    boolean forward = true;
    boolean toggleLaunch = false;

    double velocity = 1540;
    double incValue = 1;

    @Override
    public void init() {
        motor_flywheel = hardwareMap.get(DcMotorEx.class, "fly_wheel");
        motor_flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        forward = true;

        telemetry.addLine("dpad = motor direction; mode; a=start; b=stop; triggers = speed ");

        telemetry.update();
    }


    @Override
    public void loop() {
        if (toggleLaunch && forward == true) {
            motor_flywheel.setVelocity(-velocity);

        }
//        Make motor go reverse? Valid values of motors are [-1, +1]
        if(toggleLaunch && forward == false){
            motor_flywheel.setVelocity(+velocity);

        }

        if (gamepad1.a == true) {
            toggleLaunch = true;
        }

        if(gamepad1.b == true){
            toggleLaunch = false;
            motor_flywheel.setPower(0);

        }

        if (gamepad1.dpad_up == true) {
            forward = true;
        }

        if (gamepad1.dpad_down == true) {
            forward = false;
        }


        if (gamepad1.right_bumper) {
            velocity += incValue;
        }

        if (gamepad1.left_bumper) {
            velocity -= incValue;
        }

        velocity = Range.clip(velocity, 0, 5000);

        update_telemetry();
    }

    public void update_telemetry () {
        telemetry.addLine("dpad = motor direction; mode; a=start; b=stop; bumpers = speed ");
        telemetry.addData("Forward mode? ", forward);
        telemetry.addData("left motor power: ", motor_flywheel.getPower());
        telemetry.addData("left motor velocity: ", motor_flywheel.getVelocity());
        telemetry.addData("left motor encoders: ", motor_flywheel.getCurrentPosition());
        telemetry.addData("Velocity: ", velocity);

    }

    public void encode () {
        motor_flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
}
