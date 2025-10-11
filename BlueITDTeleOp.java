package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Odometry.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Robots.ITDBot;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Sensors.ColorDistSensor;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Sensors.RGBLight;

@TeleOp (name = "ITD Bot TeleOp", group = "Drive")
public class BlueITDTeleOp extends OpMode {

    double leftStickYVal;
    double leftStickXVal;
    double rightStickYVal;
    double rightStickXVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold;
    double speedMultiply = 0.75;

    private static final int PROFILE_1 = 1;  //Charlie
    private static final int PROFILE_2 = 2;  // Evan
    private int currentProfile = PROFILE_2;
    //public double mechanismPower = ___;


    public ITDBot ITDBot = new ITDBot();

    public Pinpoint odo = new Pinpoint();

    public RGBLight led = new RGBLight();

    public ColorDistSensor sensor = new ColorDistSensor();

    public RGBLight.ColorOptions color = RGBLight.ColorOptions.OFF;

    public double redMath = 0;
    public double blueMath = 0;
    public double greenMath = 0;
    public double hueMath = 0;

    public IntakeState intakeState = IntakeState.READY;
    public OuttakeState outtakeState = OuttakeState.READY;
    public SampleDumpState sampleDumpState = SampleDumpState.READY;
    public SampleResetState sampleResetState = SampleResetState.READY;

    ElapsedTime timer = new ElapsedTime();


    //double botHeading = ITDBot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    // Declare a Servo object
    public Servo intakeHolderFlip = null;

    // Variables for tracking servo position and movement
    public double currentPosition = 0.0;
    public double targetPosition = 0.17;  // Target position (1.0 is fully extended)
    public double increment = 0.005;  // How much to increment the servo position each loop
    public ElapsedTime runtime = new ElapsedTime();
    private boolean moving = false;  // State variable to track if servo is moving


    @Override
    public void init() {
        ITDBot.initRobot(hardwareMap);
        odo.initPinpoint(hardwareMap);
        led.initRGBLight(hardwareMap);
        sensor.initColorDistSensor(hardwareMap);
       // resetHeading();                       // PINPOINT
       ITDBot.imu.resetYaw();                   // REV
    }


    public void start() {
        led.setColor(RGBLight.ColorOptions.OFF);
    }


    @Override
    public void loop() {
        speedControl();
        telemetryOutput();
        preventClawOnStart();
        //slowIntake();
        bucketControl();
        intakeControl();
        intakeHolderFlipControl();
        drive();
        colorSensorControl();
        sampleResetStateControl();
        scoreSampleControl();
        intakePrepControl();
        outtakeControl();
        statesControl();
        //fieldCentricDrive();
        //imuStart();
        //driveCases();
        //transferControl();
    }

    @Override
    public void stop() {
        // Make sure to stop the servo or reset it if needed
        ITDBot.intakeHolderFlip.setPosition(0.37);  // Reset the servo to its starting position
    }



    // ********* TeleOp Control Methods **************


    // ****** Helper Method to reset Pinpoint Heading
    public void resetHeading() {
        odo.reset();
        Pose2D pos = odo.getPosition();

        pos.getHeading(AngleUnit.DEGREES);
        odo.update();
    }

    // ****** Helper Method to get Pinpoint Heading

    public double getHeading() {
        odo.update();
        Pose2D pos = odo.getPosition();
        // YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return pos.getHeading(AngleUnit.DEGREES);
    }


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
            ITDBot.imu.resetYaw();
        }
    }

    //*********  Driver 1 and Driver 2 Control Methods

    public void preventClawOnStart() {
        if (gamepad1.a) {
            ITDBot.stopMotors();
        }
    }

    public void bucketControl(){
        if (gamepad2.dpad_right) {
            ITDBot.emptyBucket();
        }
        if (gamepad2.dpad_left) {
            ITDBot.fillBucket();
        }

        if (gamepad2.right_stick_y > 0.1) {
            ITDBot.bucketSlideDown(1);

        } else if (gamepad2.right_stick_y < -0.1) {
            ITDBot.bucketSlideUp(1);
        }

        else if(sampleDumpState != SampleDumpState.BUCKET_EXTEND && sampleResetState != SampleResetState.BUCKET_RETRACT){
            ITDBot.bucketSlideStop();
        }
    }


    public void intakeControl(){

        if(gamepad2.right_bumper){
            ITDBot.sampleIntake();
        }

        else if(gamepad2.left_bumper){
            ITDBot.sampleOuttake();
        }
        else if(gamepad2.start){
            ITDBot.intakeStop();
        }


//        if(gamepad2.y){
//            ITDBot.extendIntake();
//        }
        if (gamepad2.back) {
            ITDBot.retractIntake();
        }
        if(gamepad2.dpad_down){
            ITDBot.neutralIntake();
        }

        if(gamepad2.left_stick_y>0.1){
            ITDBot.retractIntakeManual();
        }
        if(gamepad2.left_stick_y<-0.1){
            ITDBot.extendIntakeManual();
        }


    }

    public void intakeHolderFlipControl(){

        if (gamepad2.left_trigger > 0.1){
            ITDBot.scoreIntake();
        }
        if (gamepad2.right_trigger > 0.1){
            ITDBot.collectIntake();
        }
        if (gamepad2.dpad_up){
            ITDBot.submersibleIntake();
        }

    }

   public void statesControl(){
       if (gamepad2.right_stick_button ) {
           outtakeState = OuttakeState.READY;
           sampleDumpState = SampleDumpState.READY;
           intakeState = IntakeState.READY;
           sampleResetState = SampleResetState.READY;

       }

        if(gamepad2.a){
           outtakeState = OuttakeState.INTAKE_STOP;
       }
       if(gamepad2.y){
           intakeState = IntakeState.INTAKE_EXTEND;
       }

       if(gamepad2.b){
           sampleDumpState = SampleDumpState.OUTTAKE;
       }

       if(gamepad2.x){
           sampleResetState = SampleResetState.BUCKET_FILL;
       }
   }

    public enum IntakeState{
        INTAKE_EXTEND,
        WAIT,
        INTAKE_DOWN,
        INTAKE,
        READY;
    }

    public enum OuttakeState{
        INTAKE_STOP,
        INTAKE_UP,
        WAIT,
        INTAKE_RETRACT,
        READY;
    }

    public enum SampleDumpState{
        OUTTAKE,
        INTAKE_STOP,
        INTAKE_EXTEND,
        WAIT,
        WAIT2,
        BUCKET_EXTEND,
        BUCKET_DUMP,
        BUCKET_STOP,
        READY;

    }

    public enum SampleResetState{
        BUCKET_FILL,
        BUCKET_RETRACT,
        INTAKE_RETRACT,
        BUCKET_STOP,
        READY;
    }

    public void intakePrepControl() {
        switch (intakeState) {
            case INTAKE_EXTEND:
                ITDBot.extendIntake();
                timer.reset();
                intakeState = IntakeState.INTAKE_DOWN;
                break;

            case WAIT:
                if (timer.time() > 1.0) {
                    intakeState = IntakeState.INTAKE_DOWN;
                }
                break;

            case INTAKE_DOWN:
                ITDBot.collectIntake();
                intakeState = IntakeState.READY;
                break;

            case INTAKE:
                ITDBot.sampleIntake();
                intakeState = IntakeState.READY;
                break;

            case READY:
                break;
        }
    }

    public void outtakeControl(){
        switch(outtakeState){
            case INTAKE_STOP:
                ITDBot.intakeStop();
                outtakeState = OuttakeState.INTAKE_UP;
                break;
            case INTAKE_UP:
                ITDBot.scoreIntake();
                timer.reset();
                outtakeState = OuttakeState.INTAKE_RETRACT;
                break;

            case WAIT:
                if(timer.time() > 2.0){
                    outtakeState = OuttakeState.READY;
                }
                break;

            case INTAKE_RETRACT:
                ITDBot.retractIntake();
                outtakeState = OuttakeState.READY;
                break;

            case READY:
                break;
        }
    }

    public void scoreSampleControl(){
        switch(sampleDumpState){
            case OUTTAKE:
                ITDBot.sampleOuttake();
                timer.reset();
                sampleDumpState = SampleDumpState.INTAKE_STOP;
                break;

            case WAIT:
                if(timer.time() > 1.0){
                    sampleDumpState = SampleDumpState.INTAKE_STOP;
                }
                break;

            case INTAKE_STOP:
                ITDBot.intakeStop();
                sampleDumpState = SampleDumpState.INTAKE_EXTEND;
                break;

            case INTAKE_EXTEND:
                ITDBot.neutralIntake();
                timer.reset();
                sampleDumpState = SampleDumpState.BUCKET_EXTEND;
                break;

            case  BUCKET_EXTEND:
               if(timer.time() > 1.0){
                    ITDBot.bucketSlideUp(1);
                }
                if(timer.time() > 2.5){
                    sampleDumpState = SampleDumpState.BUCKET_STOP;
                }
                break;

            case BUCKET_STOP:
                ITDBot.bucketSlideStop();
                sampleDumpState = SampleDumpState.READY;
                break;

            case BUCKET_DUMP:
                ITDBot.emptyBucket();
                sampleDumpState = SampleDumpState.READY;
                break;

            case READY:
                break;
        }
    }
    public void sampleResetStateControl(){
        switch(sampleResetState){
            case BUCKET_FILL:
                ITDBot.fillBucket();
                timer.reset();
                sampleResetState = SampleResetState.BUCKET_RETRACT;
                break;

            case BUCKET_RETRACT:
                ITDBot.bucketSlideDown(1);
                if (timer.time() > 1.75){
                    sampleResetState = SampleResetState.BUCKET_STOP;
                }
                break;

            case BUCKET_STOP:
                ITDBot.bucketSlideStop();
                sampleResetState = SampleResetState.INTAKE_RETRACT;
                break;

            case INTAKE_RETRACT:
                ITDBot.retractIntake();
                sampleResetState = SampleResetState.READY;
                break;

            case READY:
                break;
        }
    }

    public void IntakeAssistControl () {
//        Take out this conditional and leave just "gamepad2.left_trigger > 0.5 " if D2 wants to be able to retract no matter waht.
//         && ITDBot.intakeHolderFlip.getPosition() >= 0.6
        if (gamepad2.left_bumper && ITDBot.intakeHolderFlip.getPosition() >= 0.6) {
            telemetry.addLine("SAMPLE INTAKE TO BUCKET CONTROL");
            ITDBot.SampleIntakeToBucket();
        }
    }

    public void transferControl(){
        if(gamepad2.back){
            ITDBot.scoringTransferStart();
        }
    }

    public void slowIntake(){
        currentPosition = ITDBot.intakeHolderFlip.getPosition();
            if (moving) {
                // Gradually move the servo to the target position in small increments
                if (Math.abs(currentPosition - targetPosition) > increment) {
                    if (currentPosition < targetPosition) {
                        currentPosition += increment;
                    } else {
                        currentPosition -= increment;
                    }
                    ITDBot.intakeHolderFlip.setPosition(currentPosition);  // Set the servo's position
                } else {
                    // Once we're close to the target, stop     and set the servo position exactly
                    currentPosition = targetPosition;
                    ITDBot.intakeHolderFlip.setPosition(currentPosition);
                    moving = false;  // Stop the movement
                }
            }

            // For example, you can trigger the servo movement when the 'A' button is pressed
            if (gamepad2.y) {
                // Start the movement towards the target position
                moving = true;
            }

            // You can use telemetry to visualize the current servo position
            telemetry.addData("Servo Position", currentPosition);
            telemetry.addData("Moving", moving ? "Yes" : "No");
            telemetry.update();
    }


    public void colorSensorControl(){

        sensor.convertColors();

        redMath = sensor.colorSensor.red();
        blueMath = sensor.colorSensor.blue();
        greenMath = sensor.colorSensor.green();
        hueMath = sensor.hsvValues[0];

        if(hueMath <30){
            led.setColor(RGBLight.ColorOptions.RED);
        }

        else if (hueMath > 200){
            led.setColor(RGBLight.ColorOptions.BLUE);
        }
        else if(hueMath > 30 && hueMath < 90){
            led.setColor(RGBLight.ColorOptions.YELLOW);
        }
        else{
            led.setColor(RGBLight.ColorOptions.OFF);
        }
    }

    // ********  Legacy Drive Control Methods

    public void fieldCentricDrive() {
        if (gamepad1.options) {
            ITDBot.imu.resetYaw();
        }
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            ITDBot.imu.resetYaw();
        }

        double botHeading = ITDBot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        ITDBot.frontLeftMotor.setPower(frontLeftPower);
        ITDBot.rearLeftMotor.setPower(backLeftPower);
        ITDBot.frontRightMotor.setPower(frontRightPower);
        ITDBot.rearRightMotor.setPower(backRightPower);
    }


    // Robot Centric Drive Method
    public void drive() {

        // Joystick values
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
            // Name of Driver using Profile 2
            case PROFILE_2: //Audrey
                //leftStickXVal controls strafing, and rightStickXVal controls rotation.
                frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
                frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
                rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
                rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
                break;

            // Default Driver Profile
            default:
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
        setMotorPower(ITDBot.frontLeftMotor, frontLeftSpeed, powerThreshold, speedMultiply);
        setMotorPower(ITDBot.frontRightMotor, frontRightSpeed, powerThreshold, speedMultiply);
        setMotorPower(ITDBot.rearLeftMotor, rearLeftSpeed, powerThreshold, speedMultiply);
        setMotorPower(ITDBot.rearRightMotor, rearRightSpeed, powerThreshold, speedMultiply);
    }

    // ***** Field Centric Drive
    public void fieldCentricDrivePinpoint(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.y) {
            resetHeading();
            getHeading();
            //ITDBot.imu.resetHeading();
        }

        double botHeading = getHeading();
        //double botHeading = odo.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        ITDBot.frontLeftMotor.setPower(frontLeftPower);
        ITDBot.rearLeftMotor.setPower(backLeftPower);
        ITDBot.frontRightMotor.setPower(frontRightPower);
        ITDBot.rearRightMotor.setPower(backRightPower);
    }


    // ***** Helper Method for Telemetry
    public void telemetryOutput() {
        telemetry.addData("Heading: ",  getHeading());
        telemetry.addData("Current X Pos: ", odo.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Current Y Pos: ", odo.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("RGB Light Color: ", color);
        telemetry.addData("Red Value: ", sensor.colorSensor.red());
        telemetry.addData("Green Value: ", sensor.colorSensor.green());
        telemetry.addData("Blue Value: ", sensor.colorSensor.blue());
        //telemetry.addData("Hue Value: ", sensor.hsvValues[0]);
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


// Field Centric Drive using Rev Robotics Control Hub
//    public void fieldCentricDrive(){
//        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;
//
//        // This button choice was made so that it is hard to hit on accident,
//        // it can be freely changed based on preference.
//        // The equivalent button is start on Xbox-style controllers.
//
//        double botHeading = ITDBot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        double frontLeftPower = (rotY + rotX + rx) / denominator;
//        double backLeftPower = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower = (rotY + rotX - rx) / denominator;
//
//        ITDBot.frontLeftMotor.setPower(frontLeftPower);
//        ITDBot.rearLeftMotor.setPower(backLeftPower);
//        ITDBot.frontRightMotor.setPower(frontRightPower);
//        ITDBot.rearRightMotor.setPower(backRightPower);
//    }

}