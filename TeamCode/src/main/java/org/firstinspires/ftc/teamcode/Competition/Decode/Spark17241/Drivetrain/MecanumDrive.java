package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.DecodeBot;

public class MecanumDrive extends DecodeBot {


    public MecanumDrive() {

    }


    // Helper Method for Linear Op
    public LinearOpMode LinearOp = null;

    public void setLinearOp(LinearOpMode LinearOp) {
        this.LinearOp = LinearOp;
    }

    //******  Methods using IMU / Gyro  **************

    // Helper Method to Get Heading using IMU
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);


    }

    // Helper Method to reset the IMU Yaw Heading
    public void resetHeading() {
        imu.resetYaw();
    }

    // ************** Basic Drive Method in Autonomous overflow 1 ***********************

    public void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
    }

    public void driveForward(double speed) {
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        rearLeftMotor.setPower(-speed);
        rearRightMotor.setPower(-speed);
    }

    public void driveBack(double speed) {
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);
    }

    public void rotateLeft(double speed) {
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(-speed);
        rearRightMotor.setPower(speed);

    }


    public void rotateRight(double speed) {
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(-speed);

    }

    public void strafeLeft(double speed) {
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(-speed);

    }

    public void strafeRight(double speed) {
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        rearLeftMotor.setPower(-speed);
        rearRightMotor.setPower(speed);
    }


    // ************** Basic Drive Method  overflow 2 ***********************

    public void driveForward(double speed, double rotations) {

        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition()) < ticks && LinearOp.opModeIsActive())) {
            driveForward(speed);
        }
        stopMotors();
    }

    public void driveBack(double speed, double rotations) {
        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition()) < ticks && LinearOp.opModeIsActive())) {
            driveBack(speed);
        }
        stopMotors();

    }

    public void strafeLeft(double speed, double rotations) {
        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition()) < ticks && LinearOp.opModeIsActive())) {
            strafeLeft(speed);
        }
        stopMotors();
    }

    public void strafeRight(double speed, double rotations) {
        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition()) < ticks && LinearOp.opModeIsActive())) {
            strafeRight(speed);
//            LinearOp.telemetry.addData("fl motor ticks", frontLeftMotor.getCurrentPosition());
//            LinearOp.telemetry.update();

        }
        stopMotors();

    }

    public void rotateRight(double speed, double rotations) {
        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition()) < ticks) && LinearOp.opModeIsActive()) {
            rotateRight(speed);
        }
        stopMotors();
    }

    public void rotateLeft(double speed, double rotations) {
        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition()) < ticks) && LinearOp.opModeIsActive()) {
            rotateLeft(speed);
        }
        stopMotors();

    }
}

    // Speed Acceleration and Deceleration Method
//    public void speedAcceleration(double rotations, double maxPower, MecanumDrivePinpoint.driveDirections driveDirection) {
//        resetEncoders();
//        double targetDistance = rotations * ODO_TICKS_PER_ROTATION;
//        double accelerationDistance = targetDistance * 0.2;
//        double decelerationDistance = targetDistance * 0.7;
//        double minPowerStart = 0;
//        double minPowerStop = 0;
//        if (driveDirection == MecanumDrivePinpoint.driveDirections.DRIVE_FORWARD || driveDirection == MecanumDrivePinpoint.driveDirections.DRIVE_BACK) {
//            minPowerStart = 0.2;
//            minPowerStop = 0.2;
//        } else {
//            minPowerStart = 0.4;
//            minPowerStop = 0.4;
//        }
//
//        double power;
//        double currentDistance = 0;
//
//        while (currentDistance < targetDistance && LinearOp.opModeIsActive()) {
//
//
//            // Acceleration
//            if (currentDistance < accelerationDistance) {
//                power = maxPower * (currentDistance / accelerationDistance);
//                power = Range.clip(power, minPowerStart, maxPower);
//                LinearOp.telemetry.addData("< 0.2: ", power);
//            }
//
//            // Deceleration
//            else if (currentDistance > targetDistance - decelerationDistance) {
//                power = maxPower * ((targetDistance - currentDistance) / decelerationDistance);
//                power = Range.clip(power, minPowerStop, maxPower);
//                LinearOp.telemetry.addData("> 0.2: ", power);
//            }
//
//            // Constant Power
//            else {
//
//                power = maxPower;
//                power = Range.clip(power, minPowerStart, maxPower);
//                LinearOp.telemetry.addData("Main Drive: ", power);
//            }
//            LinearOp.telemetry.update();
//
//            // Incremental Power Assigned to Motors
//            switch (driveDirection) {
//                case STOP:
//                    stopMotors();
//                    break;
//                case DRIVE_FORWARD:
//                    driveForward(power);
//                    break;
//                case DRIVE_BACK:
//                    driveBack(power);
//                    break;
//                case STRAFE_LEFT:
//                    strafeLeft(power);
//                    break;
//                case STRAFE_RIGHT:
//                    strafeRight(power);
//                    break;
//                default:
//                    stopMotors();
//                    break;
//            }
//
//
//            try {
//                Thread.sleep(10);
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();//re-interrupt the thread
//            }
////            currentDistance = getEncoderAvgDistanceX();
//        }
//
//        stopMotors();
//
//    }

//    public void speedAccelerationStrafe(double rotations, double maxPower, MecanumDrivePinpoint.driveDirections driveDirection) {
//        double targetDistance = rotations * ODO_TICKS_PER_ROTATION;
//
//        resetEncoders();
//        double accelerationDistance = targetDistance * 0.2;
//        double decelerationDistance = targetDistance * 0.7;
//        double minPowerStart = .2;
//        double minPowerStop = 0.2;
//        double power;
//        double currentDistance = getEncoderAvgDistanceY();
//
//        while (getEncoderAvgDistanceY() < targetDistance && LinearOp.opModeIsActive()) {
//
//
//            // Acceleration
//            if (currentDistance < accelerationDistance) {
//                power = maxPower * (currentDistance / accelerationDistance);
//                power = Range.clip(power, minPowerStart, maxPower);
//                LinearOp.telemetry.addData("< 0.2: ", power);
//            }
//
//            // Deceleration
//            else if (currentDistance > targetDistance - decelerationDistance) {
//                power = maxPower * ((targetDistance - currentDistance) / decelerationDistance);
//                power = Range.clip(power, minPowerStop, maxPower);
//                LinearOp.telemetry.addData("> 0.2: ", power);
//            }
//
//            // Constant Power
//            else {
//
//                power = maxPower;
//                power = Range.clip(power, minPowerStart, maxPower);
//                LinearOp.telemetry.addData("Main Drive: ", power);
//            }
//            LinearOp.telemetry.update();
//
//            // Incremental Power Assigned to Motors
//            switch (driveDirection) {
//                case STOP:
//                    stopMotors();
//                    break;
//                case DRIVE_FORWARD:
//                    driveForward(power);
//                    break;
//                case DRIVE_BACK:
//                    driveBack(power);
//                    break;
//                case STRAFE_LEFT:
//                    strafeLeft(power);
//                    break;
//                case STRAFE_RIGHT:
//                    strafeRight(power);
//                    break;
//                default:
//                    stopMotors();
//                    break;
//            }
//
//
//            try {
//                Thread.sleep(10);
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();//re-interrupt the thread
//            }
//
//            currentDistance = getEncoderAvgDistanceY();
//        }
//
//        stopMotors();
//
//    }

    // *********  Helper methods for Encoders******************


    // Helper Method to reset encoders
//    public void resetEncoders() {
//        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    // get odo values
//    public double getLeftOdo() {
//        return leftEncoder.getCurrentPosition();
//    }
//    public double getRightOdo() {
//        return rightEncoder.getCurrentPosition();
//    }
//    public double getCenterOdo() {
//        return centerEncoder.getCurrentPosition();
//    }
//
//}
