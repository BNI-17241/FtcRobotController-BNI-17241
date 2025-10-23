package org.firstinspires.ftc.teamcode.Lab;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name = "Single Motor Velocity Tester", group="Lab")

public class SingleMotor_RunUsingEncoder extends OpMode {

    public DcMotorEx motor_flywheel = null;

    public double velocity = -500;
    public double velocity_low = -200;
    public double velocity_med = -1200;
    public double velocity_high = -2300;
    public double incValue = -1;

    @Override
    public void init() {
        motor_flywheel = hardwareMap.get(DcMotorEx.class, "left_fly_wheel");
        motor_flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public void loop() {

        if (gamepad1.a) {
            velocity = velocity_low;
        }

        if(gamepad1.b){
            velocity = velocity_med;
        }

        if(gamepad1.y){
            velocity = velocity_high;
        }

        if(gamepad1.x){
            velocity = 0;
        }

        if (gamepad1.right_bumper) {
            velocity += incValue;
        }

        if (gamepad1.left_bumper) {
            velocity -= incValue;
        }

        //velocity = Range.clip(velocity, 0, 5000);
        motor_flywheel.setVelocity(velocity);
        update_telemetry();
    }

    public void update_telemetry () {
        telemetry.addData("left motor power: ", motor_flywheel.getPower());
        telemetry.addData("left motor velocity: ", motor_flywheel.getVelocity());
        telemetry.addData("left motor encoders: ", motor_flywheel.getCurrentPosition());
        telemetry.addData("Velocity: ", velocity);

    }

}
