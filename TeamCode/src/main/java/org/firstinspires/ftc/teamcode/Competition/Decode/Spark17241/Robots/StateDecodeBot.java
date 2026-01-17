package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


public class StateDecodeBot {

    public HardwareMap hwBot = null;

    //Drivetrain Motors
    public DcMotor frontLeftMotor;//1
    public DcMotor frontRightMotor;//0
    public DcMotor rearLeftMotor;//3
    public DcMotor rearRightMotor;//2

    // Fly Wheels and Variable Feeder Motors
    public DcMotorEx launchFrontMotor;
    public DcMotorEx launchBackMotor;

    public DcMotor intakeMotor;

    public CRServo transferServo;

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


    public StateDecodeBot() {}

    //Init Method
    public void initRobot(HardwareMap hwMap) {
        hwBot = hwMap;

        //Drivetrain Motors HW Mapping
        frontLeftMotor = hwBot.dcMotor.get("front_left_motor");//Port 1 Control
        frontRightMotor = hwBot.dcMotor.get("front_right_motor");//Port 0 Control
        rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");//Port 3 Control
        rearRightMotor = hwBot.dcMotor.get("rear_right_motor");//Port 2 Control

        //Flywheels & Feed Wheel
        launchFrontMotor = hwBot.get(DcMotorEx.class, "front_launch_wheel");;//Port ex 0
        launchBackMotor = hwBot.get(DcMotorEx.class, "back_launch_wheel");;//Port ex 2

        intakeMotor = hwBot.dcMotor.get("intake_motor");

        transferServo = hwBot.crservo.get("transfer_servo");
        LED = hwBot.servo.get("led");//Servo 1 Control

        // Drivetrain Motor direction mapping
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // Drivetrain Set Motor Run Modes
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drivetrain Motor break mapping
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Flywheel & Feeder Wheel Direction Mapping
        launchFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // Flywheel & Feed Wheel Breaking
        launchFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Flywheel & Feed Wheel Encoding for Using Velocity
        launchFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //IMU for Rev Robotics Control Hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }

    public void flylaunch(double velocity ){
        launchFrontMotor.setVelocity(velocity);
        launchBackMotor.setVelocity(velocity);
    }

    public void intakeControl(double speed){
        intakeMotor.setPower(speed);
    }

    public void beginFeed(){
        //fill up with feeder logic
    }
    public void stopFeed(){
        //fill up with feeder logic
    }

    public void LEDCon(int color)
    {
        float n = new float[]{0, .279f, .333f, .388f, .5f, .611f, .722f}[color];
        LED.setPosition(n);
    }
    public void transferSpeedCon(double speed){
        transferServo.setPower(speed);
    }
}
