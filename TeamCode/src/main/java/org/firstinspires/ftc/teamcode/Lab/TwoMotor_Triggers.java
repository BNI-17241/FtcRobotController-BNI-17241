package org.firstinspires.ftc.teamcode.Lab;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Class is used for testing a single motor, using trigger for variable power.
//WITHOUT
@TeleOp(name = "two motor test - TRIGGERS", group="lab")
@Disabled

public class TwoMotor_Triggers extends OpMode {
    private DcMotor motor_left = null;
    private DcMotor motor_right = null;
    double r_trigger;
    double l_trigger;

    boolean forward = true;

    boolean toggleLaunch = false;

    double speed = 0.6;

    @Override
    public void init() {
        motor_left = hardwareMap.dcMotor.get("grabber_lift_one");
        motor_left.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_right = hardwareMap.dcMotor.get("grabber_lift_two");
        motor_right.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        forward = true;

        telemetry.addLine("Use Gamepad 2 Triggers to engage motors");
        telemetry.update();
    }


    @Override
    public void loop() {
        if (gamepad2.right_trigger > 0.1) {
            motor_left.setPower(gamepad2.right_trigger);
            motor_right.setPower(-gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > 0.1) {
            motor_left.setPower(-gamepad2.left_trigger);
            motor_right.setPower(gamepad2.left_trigger);
        }
        else {
            motor_left.setPower(0);
            motor_right.setPower(0);
        }

        update_telemetry();
    }

    public void update_telemetry () {
        telemetry.addData("left motor power: ", motor_left.getPower());
        telemetry.addData("right motor power: ", motor_right.getPower());
        telemetry.addLine("Use Gamepad 2 Triggers to engage motors");
    }
}
