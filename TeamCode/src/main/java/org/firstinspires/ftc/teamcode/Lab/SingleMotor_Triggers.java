package org.firstinspires.ftc.teamcode.Lab;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Class is used for testing a single motor, using trigger for variable power.
@TeleOp (name = "Single Motor test - Triggers to operate")
@Disabled
public class SingleMotor_Triggers extends OpMode {
    private DcMotor motor = null;
    double power;
    double powerControl = 0.7;
//    0.6 not enough from closed --> open.

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("wormgear_right");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        power = 0;

        telemetry.addLine("Right Trigger to go 'forward'");
        telemetry.addLine("Left Trigger to go 'reverse'");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger > 0.1) {
            power = powerControl;
        }
        else if (gamepad1.left_trigger > 0.1) {
            power = -powerControl;
        }
        else {
            power = 0;
        }

        if (gamepad1.b) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad2.right_bumper) {

        }

        motor.setPower(power);

        update_telemetry();
    }

    public void speedControl(){
        if (gamepad1.dpad_up) {
            motor.setPower(.25);
        }
       else if (gamepad1.dpad_right) {
           motor.setPower(.5);
        }
      else   if (gamepad1.dpad_down) {
          motor.setPower(.75);
        }
       else if (gamepad1.dpad_left) {
           motor.setPower(1);
        }
    }

    public void update_telemetry () {
        telemetry.addData("Right Trigger Value: ", gamepad1.right_trigger);
        telemetry.addData("Left Trigger Value: ", gamepad1.left_trigger);
        telemetry.addData("POWER: ", power);
        telemetry.addData("encoder pos:",motor.getCurrentPosition());
    }
}
