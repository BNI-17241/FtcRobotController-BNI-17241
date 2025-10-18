package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Pinpoint.Pinpoint;

public class DecodeBot_oz {

    public HardwareMap hwBot = null;

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor rearRightMotor;

    public DcMotor leftFlyWheel;
    public DcMotor rightFlyWheel;

    public DcMotor feederWheel;

    //public CRServo intakeServo;
    //public DcMotor intakemotor;

    public Pinpoint odo = new Pinpoint();


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


    public DecodeBot_oz() {}

    //Init Method
    public void initRobot(HardwareMap hwMap) {
        hwBot = hwMap;
        //Drivetrain Motors HW Mapping
        frontLeftMotor = hwBot.dcMotor.get("front_left_motor");//Port 0 Control
        frontRightMotor = hwBot.dcMotor.get("front_right_motor");//Port 1 Control
        rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");//Port 2 Control
        rearRightMotor = hwBot.dcMotor.get("rear_right_motor");//Port 3 Control
        //flywheels
        leftFlyWheel = hwBot.dcMotor.get("left_fly_wheel");//Port ex 0
        rightFlyWheel = hwBot.dcMotor.get("right_fly_wheel");//Port ex 1
        //feeders
        feederWheel = hwBot.dcMotor.get("feeder_wheel");//Port ex 2

        //intakeServo = hwBot.crservo.get("intake_servo");

        //intakemotor = hwBot.dcMotor.get("intake_motor");//Port ex 3

        //encoders / odo
//        leftEncoder = hwBot.dcMotor.get("left_encoder");
//        rightEncoder = hwBot.dcMotor.get("right_encoder");
//        centerEncoder = hwBot.dcMotor.get("center_encoder");


    // dirrection mapping
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFlyWheel.setDirection(DcMotor.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotor.Direction.FORWARD);

        //intakemotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        feederWheel.setDirection(DcMotor.Direction.FORWARD);

        //intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // break mapping
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intakemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //IMU for Rev Robotics Control Hub


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }

    public void flylaunch(boolean isOn, double speed){
        if (isOn == true){
            leftFlyWheel.setPower(speed);
            rightFlyWheel.setPower(speed);
        }
            else{
            leftFlyWheel.setPower(0);
            rightFlyWheel.setPower(0);
        }

    }
    public void feedArtifact(double speed){
        feederWheel.setPower(speed);
    }

    public void setIntakeServo(boolean isOn){ // currently just bool but can be changed to speed if yall want
        if (isOn){
            //intakeServo.setPower(1);
        }
        else{
            //intakeServo.setPower(0);
        }
    }
    public void setIntakemotor(boolean isOn){   // currently just bool but can be changed to speed if yall want
        if(isOn){
            //intakemotor.setPower(1);
        }
        else{
            //intakemotor.setPower(0);
        }
    }


}
