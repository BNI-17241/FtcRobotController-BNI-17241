package org.firstinspires.ftc.teamcode.Lab;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

//Class is used for testing a single motor, using trigger for variable power.
//WITHOUT
@Disabled
@TeleOp(name = "two motor launcher test - RUN USING ENCODERS", group="Lab")

public class TwoMotor_RunUsingEncoder extends OpMode {
    private DcMotorEx motor_left = null;
    private DcMotorEx motor_right = null;
    double r_trigger;
    double l_trigger;

    boolean forward = true;

    boolean toggleLaunch = false;

//    When using SetPower
//    double velocity = 0.6;
//    double incValue = 0.001;

//    When using setVelocity
    double velocity = 1540;
    double incValue = 1;

    @Override
    public void init() {
//        motor_left = hardwareMap.dcMotor.get("launcher_motor_1");
        motor_left = hardwareMap.get(DcMotorEx.class, "left_fly_wheel");
        motor_left.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right = hardwareMap.get(DcMotorEx.class, "right_fly_wheel");
        motor_right.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        forward = true;

        telemetry.addLine("dpad = motor direction; mode; a=start; b=stop; triggers = speed ");

        telemetry.update();
    }


    @Override
    public void loop() {
        if (toggleLaunch && forward == true) {
            motor_left.setVelocity(-velocity);
            motor_right.setVelocity(-velocity);

        }
// 1)
//        Make motor go reverse? Valid values of motors are [-1, +1]
        if(toggleLaunch && forward == false){
            motor_left.setVelocity(+velocity);
            motor_right.setVelocity(+velocity);
        }

        if (gamepad1.a == true) {
            toggleLaunch = true;
        }

        if(gamepad1.b == true){
            toggleLaunch = false;
            motor_left.setPower(0);
            motor_right.setPower(0);
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

        //    When using SetPower
//        velocity = Range.clip(velocity, 0, 1);
        //    When using setVelocity
        velocity = Range.clip(velocity, 0, 5000);


        update_telemetry();
    }

    public void update_telemetry () {
        telemetry.addLine("dpad = motor direction; mode; a=start; b=stop; bumpers = speed ");
        telemetry.addData("Forward mode? ", forward);
//        telemetry.addData("A being pressed? ", gamepad1.a);
        telemetry.addData("left motor power: ", motor_left.getPower());
        telemetry.addData("right motor power: ", motor_right.getPower());
        telemetry.addData("left motor velocity: ", motor_left.getVelocity());
        telemetry.addData("right motor velocity: ", motor_right.getVelocity());
        telemetry.addData("left motor encoders: ", motor_left.getCurrentPosition());
        telemetry.addData("right motor encoders: ", motor_right.getCurrentPosition());
        telemetry.addData("Velocity: ", velocity);

    }

    public void encode () {
        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
}
