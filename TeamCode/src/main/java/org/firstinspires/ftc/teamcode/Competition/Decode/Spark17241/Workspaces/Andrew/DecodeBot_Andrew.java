package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Andrew;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Drivetrains.MecanumDrive;

public class DecodeBot_Andrew extends MecanumDrive {

    public HardwareMap hwBot = null;

    public Limelight3A limelight = null;

    //Drivetrain Motors
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor rearRightMotor;

    // Fly Wheels and Variable Feeder Motors
    public DcMotorEx leftFlyWheel;
    public DcMotorEx rightFlyWheel;
    public DcMotor feederWheel;
    public Servo LED;

    public LinearOpMode LinearOp = null;

    public static final double TICKS_PER_ROTATION = 386.3;
    public static final double ODO_TICKS_PER_ROTATION = 2000;

    // Instance Variables for IMU
    public IMU imu = null;
    public double headingTolerance = 0.0; //0.5
    public double currentHeading = 0;

    // Helper method to set the run modes for all motors at the same
    public void setMotorRunModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);
    }


    public DecodeBot_Andrew() {}

    //Init Method
    public void initRobot(HardwareMap hwMap) {
        hwBot = hwMap;

        //Drivetrain Motors HW Mapping
        frontLeftMotor = hwBot.dcMotor.get("front_left_motor");//Port 0 Control
        frontRightMotor = hwBot.dcMotor.get("front_right_motor");//Port 1 Control
        rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");//Port 2 Control
        rearRightMotor = hwBot.dcMotor.get("rear_right_motor");//Port 3 Control

        //Flywheels & Feed Wheel
        leftFlyWheel = hwBot.get(DcMotorEx.class, "left_fly_wheel");;//Port ex 0
        rightFlyWheel = hwBot.get(DcMotorEx.class, "right_fly_wheel");//Port ex 1
        feederWheel = hwBot.get(DcMotorEx.class,"feeder_wheel");//Port ex 2

        LED = hwBot.servo.get("led");//Servo 1 Control

        // Drivetrain Motor direction mapping
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Drivetrain Set Motor Run Modes
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drivetrain Motor break mapping
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Flywheel & Feeder Wheel Direction Mapping
        leftFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        feederWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        // Flywheel & Feed Wheel Breaking
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feederWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Flywheel & Feed Wheel Encoding for Using Velocity
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feederWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //IMU for Rev Robotics Control Hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }

    // Limelight Initialization  Methods
    public void initLimelight(HardwareMap hwMap) {
        hwBot = hwMap;
        limelight = hwBot.get(Limelight3A.class, "limelight");

    }

    // Limelight Control  Methods
    public void startLimelight() {

        limelight.pipelineSwitch(0);
        limelight.start();

    }

    public void stopLimelight() {
        limelight.stop();

    }


    // LED Control Method
    public void LEDCon(int color)
    {
        float n = new float[]{0, .279f, .333f, .388f, .5f, .611f, .722f}[color];
        LED.setPosition(n);
    }

    // Fly Wheel Control Method
    public void flylaunch(double velocity ){

        leftFlyWheel.setVelocity(velocity);
        rightFlyWheel.setVelocity(velocity);

    }

    // Feed Wheel Control Method
    public void feedArtifact(double speed){
        feederWheel.setPower(speed);

    }
}
