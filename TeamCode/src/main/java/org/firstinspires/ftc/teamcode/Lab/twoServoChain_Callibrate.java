package org.firstinspires.ftc.teamcode.Lab;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Grabber Arm - Continuous Servo")
@Disabled

public class twoServoChain_Callibrate extends OpMode {

    private Servo grabberArmLeft = null;
    private double grabberLeftArmPos = .87;
    private double incVal = 0.001;
    private CRServo grabberArmRight = null;
    private double grabberRightArmPos = 0;


    @Override
    public void init () {
//        Actual grabber server
        grabberArmLeft = hardwareMap.servo.get("grabber_arm");
        grabberArmLeft.setPosition(grabberLeftArmPos);
//        Chain / Continious rotation servo.
        grabberArmRight = hardwareMap.crservo.get("cone_sucker");
        grabberArmRight.setPower(grabberRightArmPos);
    }

    // LEFT ARM = arms servos
    // RIGHT ARM = Continious rotation servo

    @Override
    public void loop () {
        if (gamepad1.right_bumper) {
            grabberLeftArmPos += incVal;
            grabberLeftArmPos = Range.clip(grabberLeftArmPos,0,1);
            telemetry.addLine("Increase Servo Pos!");
        }

        if (gamepad1.left_bumper){
            grabberLeftArmPos -= incVal;
            grabberLeftArmPos = Range.clip(grabberLeftArmPos, 0,  1);
            telemetry.addLine( "Decrease Servo Pos!");
        }
//        close / lift
        if (gamepad1.y || gamepad2.y) {
            grabberLeftArmPos = .72;
        }
//        intake
        if (gamepad1.b || gamepad2.b) {
            grabberLeftArmPos = .71;
        }
//        open
        if (gamepad1.a || gamepad2.a) {
            grabberLeftArmPos = 0.875;
        }


        grabberArmLeft.setPosition(grabberLeftArmPos);

        if (gamepad2.right_bumper) {
            grabberArmRight.setPower(1);
            telemetry.addLine("Increase Servo Pos!");
        }

        else if (gamepad2.left_bumper){
            grabberArmRight.setPower(-1);
            telemetry.addLine( "Decrease Servo Pos!");
        }
        else {
            grabberArmRight.setPower(grabberRightArmPos);
        }

//        if (gamepad2.y) {
//            grabberRightArmPos = .4;
//        }
//        if (gamepad2.b) {
//            grabberRightArmPos = .35;
//        }
//        if (gamepad2.a) {
//            grabberRightArmPos = .32;
//        }


//        grabberArmRight.setPosition(grabberRightArmPos);
        updateTelemetry();
    }


    public void updateTelemetry () {
        telemetry.addLine("RB: increase, LB: Decrease");
        telemetry.addLine("x = set to .90, y = set to 0.10");
        telemetry.addData("Grabber Left Arm Position:", grabberArmLeft.getPosition());
        telemetry.addData("Grabber Left Arm Position:", grabberLeftArmPos);
//        telemetry.addData("Grabber Right Arm Position", grabberArmRight.getPosition());
        telemetry.addData("Grabber Right Arm Position", grabberRightArmPos);
        telemetry.update();
    }
}
