package org.firstinspires.ftc.teamcode.Lab.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp (name = "Encoder - Turntable Test - Camden", group = "LAB")


public class Encoder_Turntable_Lift_Camden extends OpMode {

    private DcMotorEx grabberLift = null;
    private DcMotorEx turretPlatform = null;

    boolean displayTelemetry = true;

    int turretClockwise = 500;
    int turretCounterclockwise = -500;
    int turretReverse = 1000;

    int liftRest = 0;
    int liftLow = 600;
    int liftMid = 1300;
    int liftHigh = 2000;

    int turretErrorMargin = 30;

    int liftLevel = 0;
    boolean liftLevelAllow = true;
    boolean liftToggle = true;
    boolean turretEncoder180 = false;
    boolean turretEncoder90CCW = false;
    boolean turretEncoder90CW = false;
    boolean turretEncoderCollect = false;

    double turretPowerEncoder = 0.3;
    double turretPowerManual = 0.1;
    double liftPowerUp = 1.0;
    double liftPowerDown = 0.8;

    public void init() {
        grabberLift = hardwareMap.get(DcMotorEx.class, "grabber_lift");
        grabberLift.setDirection(DcMotorSimple.Direction.FORWARD);
        grabberLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabberLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        No idea what values to actually use, but it definitely does affect the lift!
        grabberLift.setPositionPIDFCoefficients(10);

        turretPlatform = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretPlatform.setDirection(DcMotorSimple.Direction.REVERSE);
        turretPlatform.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretPlatform.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPlatform.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        liftControl();  // adjusts the lift level to be used in turrentMechanism
//        Moves Lift using values from liftControl()
        liftMechanismEncoder();
//        Disabled - so notes above function.
        liftMechanism();
        turretMechanism(); //Moves turret.

        if (displayTelemetry) {
            telemetryUpdate();
        }
    }

    //    Disabled - how to implement manual power control with RUN_TO_POSITION?
    public void liftMechanism() {
        if (gamepad2.dpad_up) {
            grabberLift.setPower(0.5);
        } else if (gamepad2.dpad_down) {
            grabberLift.setPower(-0.2);
        } else {
            if (grabberLift.getCurrentPosition() > 500) {
                grabberLift.setPower(0.1);
            } else {
                grabberLift.setPower(0);
            }

        }
    }

    public void liftMechanismEncoder() {
//        Only go to target position when press 'y'.
//        Allows P2 to get lift "target" position ready.
        if (gamepad2.left_stick_button) {
            switch (liftLevel) {
                case 0:
                    if (grabberLift.getCurrentPosition() > liftRest) {
                        grabberLift.setPower(liftPowerDown);
                    }
                    else {
                        grabberLift.setPower(liftPowerUp);
                    }
                    grabberLift.setTargetPosition(liftRest);
                    grabberLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
                case 1:
                    if (grabberLift.getCurrentPosition() > liftLow) {
                        grabberLift.setPower(liftPowerDown);
                    }
                    else {
                        grabberLift.setPower(liftPowerUp);
                    }
                    grabberLift.setTargetPosition(liftLow);
                    grabberLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
                case 2:
                    if (grabberLift.getCurrentPosition() > liftMid) {
                        grabberLift.setPower(liftPowerDown);
                    }
                    else {
                        grabberLift.setPower(liftPowerUp);
                    }
                    grabberLift.setTargetPosition(liftMid);
                    grabberLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
                case 3:
                    grabberLift.setPower(liftPowerUp);
                    grabberLift.setTargetPosition(liftHigh);
                    grabberLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                default:
                    break;
            }
        }
    }

    public void liftControl () {
        if (gamepad2.b == true && liftLevelAllow == true) {
            if (liftLevel < 3) {
                liftLevel += 1;
            }
            liftLevelAllow = false;
            liftToggle = false;
        }
        else if (gamepad2.x == true && liftLevelAllow == true) {
            if (liftLevel > 0) {
                liftLevel -= 1;
            }
            liftToggle = false;
            liftLevelAllow = false;
        }
        else {  // The IF here makes it so lift* goes back to default 'false' ONLY when not pressing Trigger.
            if (gamepad2.b == false && gamepad2.x == false) {
                liftLevelAllow = true;
                liftToggle = true;
            }
        }
    }


    // liftLevelAllow is what prevents the 'y' and 'b' from continuing to change liftLevel when button is held.
   /* public void liftControl () {
//        INCREASE liftLevel value
//        IF - Gamepad2 right trigger is enabled AND "liftLevelAllow" is true
        if (gamepad2.right_trigger > 0.1 && liftLevelAllow) {
//            Only allow the "liftLevel" to increase if not at top level
            if ( ) {
//                Increase liftLevel

            }
            // set "liftLevelAllow" and "liftToggle" to false!
            liftLevelAllow
            liftToggle
        }

//        DECREASE lift level allow.
//        IF - Gamepad2 left trigger is enabled AND "liftLevelAllow" is true
        else if ( ) {
//            Only allow if not at bottom level
            if () {
                liftLevel -= 1;
            }
            // set "liftLevelAllow" and "liftToggle" to false!

        }
        else {  // The IF here makes it so lift* goes back to default 'false' ONLY when not pressing Trigger.
            if (gamepad2.right_trigger < 0.1 && gamepad2.left_trigger < 0.1) {
                liftLevelAllow = true;
                liftToggle = true;
            }
        }
    }
*/
    public void turretMechanism() {

        //
        // ARE WE USING ENCODER TURN?
        //

//        IF Gamepad.x, turretEncoderCCW set to "True"
        if (gamepad2.right_bumper) {

            turretEncoder90CCW = true;

        }

        if (gamepad2.left_bumper) {

            turretEncoder90CW = true;

        }
//        Game pad.b - set turretEncoderCW to "true".
        if (gamepad2.right_trigger > 0.2) {

            turretEncoder180 = true;
//
        }

//        IF gamepad.a - set  turretEncoderCollect to True
        if (gamepad2.left_trigger > 0.2) {

            turretEncoderCollect = true;

        }

        //
        //  MANUAL CONTROL OF TURRET
        //

        // IF gamepad.left bumper ..
        if (gamepad2.a) {

            turretPlatform.setPower(-turretPowerManual);

        }
//        else if gamepad2.right bumpter
        else if (gamepad2.b) {

            turretPlatform.setPower(turretPowerManual);

        }


        //
        //  ENCODER CONTROL OF TURRET
        //

//        Check IF turretEncoderCW is "True"

        else if (turretEncoder180) {
//            If our "turretPlatform.getCurrentPosition()" is less than the "turretClockwise", turn turretPlatform
            if (turretPlatform.getCurrentPosition() >= 0) {

                if (turretPlatform.getCurrentPosition() < turretReverse) {
                    turretPlatform.setPower(+turretPowerEncoder);
                }
                else {
                    turretEncoder180 = false;
                }
            }
            else if (turretPlatform.getCurrentPosition() < 0) {
                if (turretPlatform.getCurrentPosition() > turretReverse) {
                    turretPlatform.setPower(-turretPowerEncoder);
                }
                else {
                    turretEncoder180 = false;
                }
            }
        }

//        Same concept for "turretEncoderCCW"
        else if (turretEncoder90CW) {
            if (turretPlatform.getCurrentPosition() < turretClockwise) {
                turretPlatform.setPower(+turretPowerEncoder);
            }
            else {
                turretEncoder90CW = false;
            }
        }

        else if (turretEncoder90CCW) {
            if (turretPlatform.getCurrentPosition() > turretCounterclockwise) {
                turretPlatform.setPower(-turretPowerEncoder);
            }
            else {
                turretEncoder90CCW = false;
            }
        }




//      Same concept for "turretEncoderCollect"
//      need TWO code blocks for the "turretEncoderCollect" position:
//        One if turret is on the left, second if the turret is on the right.

        else if (turretEncoderCollect) {
//            Checking if the current Turret Platform Position is on the left
//            if (turretPlatform.getCurrentPosition() > 0) {
////                Checking AGAIN if the current Turret Platform Position is > 0
//                if (turretPlatform.getCurrentPosition() > 0) {
////                    Set the turretPlatform power to "turretPowerEncoder"
//                    turretPlatform.setPower(-turretPowerEncoder);
//                } else {
////                    set "turretEncoderCollect" to "False"
//                    turretEncoderCollect = false;
//                }
//            }
//
////            Else if - same concept, but reverse (turret on right).
//            else if (turretEncoderCollect) {
//                if (turretPlatform.getCurrentPosition() < 0) {
////                    Set the turretPlatform power to "turretPowerEncoder"
//                    turretPlatform.setPower(turretPowerEncoder);
//                } else {
////                    set "turretEncoderCollect" to "False"
//                    turretEncoderCollect = false;
//                }
//            }
            if (turretPlatform.getCurrentPosition() >= 0) {
                if (turretPlatform.getCurrentPosition() >= 0) {
                    turretPlatform.setPower(-turretPowerEncoder);
                }
                else {
                    turretEncoderCollect = false;
                }

            }

            else if (turretPlatform.getCurrentPosition() < 0) {
                if (turretPlatform.getCurrentPosition() < 0) {
                    turretPlatform.setPower(+turretPowerEncoder);
                }
                else {
                    turretEncoderCollect = false;
                }
            }


            if (!turretEncoderCollect) {
                turretPlatform.setPower(0);
            }

//        Set power to 0!

        } else {
            turretPlatform.setPower(0);

        }
    }

        public void telemetryUpdate () {
            telemetry.addData("TURNTABLE ENCODERS: ", turretPlatform.getCurrentPosition());
            telemetry.addData("LIFT ENCODERS: ", grabberLift.getCurrentPosition());
            telemetry.addData("Lift Level Allow? ", liftLevelAllow);
            telemetry.addData("Current Lift Level: ", liftLevel);

            telemetry.addData("turret180 = ", turretEncoder180);
            telemetry.addData("turret90 = ", turretEncoder90CCW);
            telemetry.addData("turret0 = ", turretEncoderCollect);

        }


//    Testing.  Ignore.
        public void spdControl ( double min_spd, double max_spd, int currentPos, int targetPos){
            min_spd = 0.1;
            max_spd = 0.4;
            currentPos = 410;
            targetPos = -370;
            int travelDistance_Total = Math.abs(currentPos) + Math.abs(targetPos);
            int travelDistnce_Traversed;


        }
    }



