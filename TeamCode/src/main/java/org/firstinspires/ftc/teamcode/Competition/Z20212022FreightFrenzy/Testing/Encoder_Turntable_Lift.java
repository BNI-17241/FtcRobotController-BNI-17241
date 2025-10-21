package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp (name = "Encoder - Lift & Turntable Test", group = "LAB")


public class Encoder_Turntable_Lift extends OpMode {

    private DcMotorEx grabberLift = null;
    private DcMotorEx turretPlatform = null;

    boolean displayTelemetry = true;

    int turretClockwise = 410;
    int turretCounterclocwise = -370;

    int liftRest = 0;
    int liftLow = 600;
    int liftMid = 1300;
    int liftHigh = 2000;

    int liftLevel = 0;
    boolean liftLevelAllow = true;
    boolean liftToggle = true;
    boolean turretEncoderCW = false;
    boolean turretEncoderCCW = false;
    boolean turretEncoderCollect = false;
    boolean turrentEncoder180 = false;

    double turretPowerEncoder = 0.3;
    double turretPowerManual = 0.1;
    double liftPowerUp = 1.0;
    double liftPowerDown = 0.8;

    public void init () {
        grabberLift = hardwareMap.get(DcMotorEx.class, "grabber_lift");
        grabberLift.setDirection(DcMotorSimple.Direction.FORWARD);
        grabberLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabberLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        No idea what values to actually use, but it definitely does affect the lift!
        grabberLift.setPositionPIDFCoefficients(5);

        turretPlatform = hardwareMap.get(DcMotorEx.class,"turret_motor");
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
//        liftMechanism();
        turretMechanism(); //Moves turret.

        if (displayTelemetry) {
            telemetryOuput();
        }
    }

//    Disabled - how to implement manual power control with RUN_TO_POSITION?
    public void liftMechanism () {
        if (gamepad2.dpad_up) {
            grabberLift.setPower(0.5);
        }
        else if (gamepad2.dpad_down) {
            grabberLift.setPower(-0.2);
        }
        else {
            if (grabberLift.getCurrentPosition() > 500) {
                grabberLift.setPower(0.1);
            }
            else {
                grabberLift.setPower(0);
            }

        }
    }

    public void liftMechanismEncoder () {
//        Only go to target position when press 'y'.
//        Allows P2 to get lift "target" position ready.
        if (gamepad2.y) {
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


    // liftLevelAllow is what prevents the 'y' and 'b' from continuing to change liftLevel when button is held.
    public void liftControl () {
        if (gamepad2.right_trigger > 0.1 && liftLevelAllow) {
            if (liftLevel < 3) {
                liftLevel += 1;
            }
            liftLevelAllow = false;
            liftToggle = false;
        }
        else if (gamepad2.left_trigger > 0.1 && liftLevelAllow) {
            if (liftLevel > 0) {
                liftLevel -= 1;
            }
            liftToggle = false;
            liftLevelAllow = false;
        }
        else {  // The IF here makes it so lift* goes back to default 'false' ONLY when not pressing Trigger.
            if (gamepad2.right_trigger < 0.1 && gamepad2.left_trigger < 0.1) {
                liftLevelAllow = true;
                liftToggle = true;
            }
        }
    }

    public void turretMechanism () {
        int currentTurretEncoder = turretPlatform.getCurrentPosition();

        //
        // ARE WE USING ENCODER TURN?
        //

        if (gamepad2.x) {
            turretEncoderCCW = true;
        }
        if (gamepad2.b) {
            turretEncoderCW = true;
        }
        if (gamepad2.a) {
            turretEncoderCollect = true;
        }
        if (gamepad2.y) {
            turrentEncoder180 = true;
        }

        //
        //  MANUAL CONTROL OF TURRET
        //

        if (gamepad2.left_bumper) {
            turretPlatform.setPower(-turretPowerManual);
        }
        else if (gamepad2.right_bumper) {
            turretPlatform.setPower(+turretPowerManual);
        }


        //
        //  ENCODER CONTROL OF TURRET
        //

        else if (turretEncoderCW) {
            if (turretPlatform.getCurrentPosition() < turretClockwise) {
                turretPlatform.setPower(+turretPowerEncoder);
            }
            else {
                turretEncoderCW = false;
            }
        }

        else if (turretEncoderCCW) {
            if (turretPlatform.getCurrentPosition() > turretCounterclocwise) {
                turretPlatform.setPower(-turretPowerEncoder);
            }
            else {
                turretEncoderCCW = false;
            }
        }


        else if (turretEncoderCollect) {
//            if (turretPlatform.getCurrentPosition() > 0) {
//                if (turretPlatform.getCurrentPosition() > 0) {
//                    turretPlatform.setPower(-turretPowerEncoder);
//                }
//                else {
//                    turretEncoderCollect = false;
//                }
//            }
//
//            else if (turretPlatform.getCurrentPosition() < 0) {
//                if (turretPlatform.getCurrentPosition() < 0) {
//                    turretPlatform.setPower(+turretPowerEncoder);
//                }
//                else {
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

        }
        else if (turrentEncoder180 == true) {
            if (turretPlatform.getCurrentPosition() >= 0) {

                if (turretPlatform.getCurrentPosition() < turretClockwise*2) {
                    turretPlatform.setPower(+turretPowerEncoder);
                }
                else {
                    turrentEncoder180 = false;
                }
            }
            else if (turretPlatform.getCurrentPosition() < 0) {
                if (turretPlatform.getCurrentPosition() > turretCounterclocwise*2) {
                    turretPlatform.setPower(-turretPowerEncoder);
                }
                else {
                    turrentEncoder180 = false;
                }
            }
        }
        else {
            turretPlatform.setPower(0);
            turrentEncoder180 = false;
        }
    }

    public void telemetryOuput () {
        telemetry.addData("TURNTABLE ENCODERS: ", turretPlatform.getCurrentPosition());
        telemetry.addData("LIFT ENCODERS: ", grabberLift.getCurrentPosition());
        telemetry.addData("Lift Level Allow? ", liftLevelAllow);
        telemetry.addData("Current Lift Level: ", liftLevel);
    }


//    Testing.  Ignore.
    public void spdControl (double min_spd, double max_spd, int currentPos, int targetPos) {
        min_spd = 0.1;
        max_spd = 0.4;
        currentPos = 410;
        targetPos = -370;
        int travelDistance_Total = Math.abs(currentPos) + Math.abs(targetPos);
        int travelDistnce_Traversed;


    }
}
