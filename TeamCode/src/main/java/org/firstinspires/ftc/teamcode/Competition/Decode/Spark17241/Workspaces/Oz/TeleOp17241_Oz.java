package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.DecodeBot_oz;
import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Oz.TeleOp17241_Oz;

@TeleOp(name = "DecodeBot Oz", group = "Lab")
public class TeleOp17241_Oz extends OpMode {


    double leftStickYVal;
    double leftStickXVal;
    double rightStickYVal;
    double rightStickXVal;
//
    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold;
    double speedMultiply = 0.75;

    private static final int PROFILE_1 = 1;  //User 1
    private static final int PROFILE_2 = 2; //user 2
    private int currentProfile = PROFILE_1;

    public DecodeBot_oz decBot = new DecodeBot_oz();

    public GoBildaPinpointDriver odo = null;

    //double botHeading = decBot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    ElapsedTime timer = new ElapsedTime();
    public enum singleFeedStates {
        READY,
        START,
        PAUSE,
        STOP,
    }

    public TeleOp17241_Oz.singleFeedStates singleFeedState = TeleOp17241_Oz.singleFeedStates.READY;

    public enum multipleFeedStates {
        READY,
        // First Feed into Fly Wheels
        START_1,
        PAUSE_1,
        STOP_1,
        WAIT_1,
        // Second Feed into Fly Wheels
        START_2,
        PAUSE_2,
        STOP_2,
        WAIT_2,
        // Third Feed into Fly Wheels
        START_3,
        PAUSE_3,
        STOP_3,
    }

    public TeleOp17241_Oz.multipleFeedStates multipleFeedState = TeleOp17241_Oz.multipleFeedStates.READY;

    @Override
    public void init() {
        decBot.initRobot(hardwareMap);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // resetHeading();                       // PINPOINT
        decBot.imu.resetYaw();                   // REV
    }

    @Override
    public void loop() {
        speedControl();
        telemetryOutput();
        RobotCentricDrive();
        flyWheelControl();
        feedStateController();
        feedStatesSingleLoad();
        feedStatesMultipleLoad();

        //fieldCentricDrive();
        imuStart();
        //driveCases();
        //transferControl();
    }





//    // ********* TeleOp Control Methods **************

    // ****** Helper method to set Motor Power
    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }
    public void imuStart(){
        if(gamepad1.options){
            decBot.imu.resetYaw();
        }
    }

    //*********  Driver 1 and Driver 2 Control Methods

    // Robot Centric Drive Method
    public void RobotCentricDrive() {
        //reverse drive
        leftStickYVal = -gamepad1.left_stick_y;

        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
        rightStickYVal = gamepad1.right_stick_y;
        rightStickYVal = Range.clip(rightStickYVal, -1, 1);

        leftStickXVal = gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = gamepad1.right_stick_x;
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);

        switch (currentProfile) {
            // Name of Driver using Profile 1
            case PROFILE_1:
                // leftStickXVal controls rotation, and rightStickXVal controls strafing.
                frontLeftSpeed = leftStickYVal + rightStickXVal + leftStickXVal;    // Vertical + Rotation + Staffing
                frontRightSpeed = leftStickYVal - rightStickXVal - leftStickXVal;   // Vertical - Rotation - Strafing(sign in front is the way the motor is turning in relation to the others)
                rearLeftSpeed = leftStickYVal - rightStickXVal + leftStickXVal;
                rearRightSpeed = leftStickYVal + rightStickXVal - leftStickXVal;
                break;
            case PROFILE_2:
                // This is copy of profile one
                frontLeftSpeed = leftStickYVal + rightStickXVal + leftStickXVal;    // Vertical + Rotation + Staffing
                frontRightSpeed = leftStickYVal - rightStickXVal - leftStickXVal;   // Vertical - Rotation - Strafing(sign in front is the way the motor is turning in relation to the others)
                rearLeftSpeed = leftStickYVal - rightStickXVal + leftStickXVal;
                rearRightSpeed = leftStickYVal + rightStickXVal - leftStickXVal;

            // Default Driver Profile
            default:
                // stops robot
                frontLeftSpeed = 0;
                frontRightSpeed = 0;
                rearLeftSpeed = 0;
                rearRightSpeed = 0;
                break;
        }

        // Clipping motor speeds to [-1, 1]
        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);
        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

        // Setting motor powers (with threshold check)
        setMotorPower(decBot.frontLeftMotor, frontLeftSpeed, powerThreshold, speedMultiply);
        setMotorPower(decBot.frontRightMotor, frontRightSpeed, powerThreshold, speedMultiply);
        setMotorPower(decBot.rearLeftMotor, rearLeftSpeed, powerThreshold, speedMultiply);
        setMotorPower(decBot.rearRightMotor, rearRightSpeed, powerThreshold, speedMultiply);
    }

    // ***** Helper Method for Telemetry
    public void telemetryOutput() {
       // telemetry.addData("Heading: ",  getHeading());
       // telemetry.addData("Current X Pos: ", odo.getPosition().getX(DistanceUnit.INCH));
       // telemetry.addData("Current Y Pos: ", odo.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("Single Feed State: ", singleFeedState);
        telemetry.addData("Multiple Feed State: ", multipleFeedState);
        telemetry.addData("Left Fly Wheel: ", decBot.leftFlyWheel.getPower());
        telemetry.addData("Right Fly Wheel: ", decBot.rightFlyWheel.getPower());


        telemetry.addData("get Position", odo.getPosition());


        telemetry.update();
    }
    // ***** Helper Method for Speed Control
    public void speedControl() {
        if (gamepad1.dpad_up) {
            speedMultiply = 0.5;
        } else if (gamepad1.dpad_right) {
            speedMultiply = 0.75;
        } else if (gamepad1.dpad_down) {
            speedMultiply = 0.25;
        } else if (gamepad1.dpad_left) {
            speedMultiply = 1;
        }
    }


    //************ Control surface interfaces******************

    public void flyWheelControl(){

        if(gamepad1.x){
            decBot.flylaunch(true, .2);
        }
        else if(gamepad1.a){
            decBot.flylaunch(true, .4);
        }
        else if(gamepad1.b){
            decBot.flylaunch(true, .6);
        }

        if(gamepad1.right_bumper){
            decBot.flylaunch(false, 0);}
    }


    // Feed Controller using States
    public void feedStateController() {
        if (gamepad1.left_trigger > 0.5) {
            singleFeedState = TeleOp17241_Oz.singleFeedStates.START;
        }
        else if (gamepad1.right_trigger > 0.5)  {
            multipleFeedState = TeleOp17241_Oz.multipleFeedStates.START_1;
        }
    }

    // State Method for Single Feeding into fly wheels
    public void feedStatesSingleLoad() {
        switch (singleFeedState) {
            case START:
                decBot.feedArtifact( 1.0);
                timer.reset();
                singleFeedState = TeleOp17241_Oz.singleFeedStates.PAUSE;
                break;
            case PAUSE:
                if (timer.time() > 0.5) {
                    singleFeedState = TeleOp17241_Oz.singleFeedStates.STOP;
                }
                break;
            case STOP:
                decBot.feedArtifact( 0.0);
                singleFeedState = TeleOp17241_Oz.singleFeedStates.READY;
                break;
            case READY:
                break;
        }
    }

    // State Method for Multiple Feeding into fly wheels
    public void feedStatesMultipleLoad() {
        switch (multipleFeedState) {
            case START_1:
                decBot.feedArtifact( 1.0);
                timer.reset();
                multipleFeedState = TeleOp17241_Oz.multipleFeedStates.PAUSE_1;
                break;
            case PAUSE_1:
                if (timer.time() > 0.5) {
                    multipleFeedState = TeleOp17241_Oz.multipleFeedStates.STOP_1;
                }
                break;
            case STOP_1:
                decBot.feedArtifact( 0.0);
                timer.reset();
                multipleFeedState = TeleOp17241_Oz.multipleFeedStates.WAIT_1;
                break;
            case WAIT_1:
                if (timer.time() > 0.5) {
                    multipleFeedState = TeleOp17241_Oz.multipleFeedStates.START_2;
                }
                break;
            case START_2:
                decBot.feedArtifact( 1.0);
                timer.reset();
                multipleFeedState = TeleOp17241_Oz.multipleFeedStates.PAUSE_2;
                break;
            case PAUSE_2:
                if (timer.time() > 0.5) {
                    multipleFeedState = TeleOp17241_Oz.multipleFeedStates.STOP_2;
                }
                break;
            case STOP_2:
                decBot.feedArtifact( 0.0);
                timer.reset();
                multipleFeedState = TeleOp17241_Oz.multipleFeedStates.WAIT_2;
                break;
            case WAIT_2:
                if (timer.time() > 0.50) {
                    multipleFeedState = TeleOp17241_Oz.multipleFeedStates.START_3;
                }
                break;
            case START_3:
                decBot.feedArtifact( 1.0);
                timer.reset();
                multipleFeedState = TeleOp17241_Oz.multipleFeedStates.PAUSE_3;
                break;
            case PAUSE_3:
                if (timer.time() > 0.5) {
                    multipleFeedState = TeleOp17241_Oz.multipleFeedStates.STOP_3;
                }
                break;
            case STOP_3:
                decBot.feedArtifact( 0.0);
                timer.reset();
                multipleFeedState = TeleOp17241_Oz.multipleFeedStates.READY;
                break;
            case READY:
                break;
        }
    }
public void intake(){
        if(gamepad1.y){
            decBot.setIntakeServo(true); // prob do toogle later-oz
            decBot.setIntakemotor(true);
        }
        else{
            decBot.setIntakeServo(false);
            decBot.setIntakemotor(true);
        }
}


}