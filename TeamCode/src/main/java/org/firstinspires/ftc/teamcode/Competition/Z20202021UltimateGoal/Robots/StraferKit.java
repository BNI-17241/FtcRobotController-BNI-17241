package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.DriveTrains.MecanumDrive;

public class StraferKit extends MecanumDrive {
    public HardwareMap hwBot = null;

//    public Servo flag = null;

//GYRO INITIALIZATION

    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public final double SPEED = .3;
    public final double TOLERANCE = .4;

    // Color and Distance Hardware & Variables
//    public ColorSensor sensorColor;
//    public DistanceSensor sensorDistance;
    public float hsvValues[] = {0F, 0F, 0F};
    public final double SCALE_FACTOR = 1;
    //  Camera Initialization
    /*
    public OpenCvCamera webcam;
    public SkystoneDeterminationPipeline pipeline;

     */

//    public static final double TICKS_PER_ROTATION = 383.6;   // GoBilda 13.7 Motor PPR



/*
    Servo WobbleArm = null;
    Servo WobbleGrab = null;
    Servo Camera =null;
 */

//    Servos servos = new Servos();


    /*
        public double servoOpenPos = 0.36;
        public double servoClosePos = 0.93;
        //    was 0.446
        public double WobbleArmRaisedPos = 0.23;
        public double WobbleArmLowerPos = 0.613;
        public double WobbleGrabOpenPos = 0.651;
        public double WobbleGrabClosePos = 0.312;

     */
    //Blue Left:
//    public double CameraServoPosBlueLeft = 0.358;
    //Blue Right:
//    public double CameraServoPosBlueRight = 0.602;
    //Launcher Motor:

    /*
    public DcMotor LauncherMotor = null;
    public DcMotor IntakeMotor = null;
     */

//    private final static int    LED_PERIOD = 10;
    private final static int GAMEPAD_LOCKOUT = 500;

//    public RevBlinkinLedDriver blinkinLedDriver;
//    public RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }
    //LabBot constructor
    public StraferKit() {

    }
    public void initRobot(HardwareMap hwMap){
        hwBot = hwMap;
        /*
        WobbleArm = hwBot.get(Servo.class, "wobble_arm");
        WobbleArm.setDirection(Servo.Direction.FORWARD);
        if (mode.equals("auto")) {
            WobbleArm.setPosition(WobbleArmRaisedPos);

        }


        WobbleGrab = hwBot.get(Servo.class, "wobble_grab");
        WobbleGrab.setDirection(Servo.Direction.FORWARD);
        if (mode.equals("auto")){
            WobbleGrab.setPosition(WobbleGrabClosePos);
        }
        Camera = hwBot.get(Servo.class, "camera_blue_left_servo");
        Camera.setDirection(Servo.Direction.FORWARD);

        switch (startPosition) {
            case "BlueLeft":
                Camera.setPosition(CameraServoPosBlueLeft);
                break;
            case "BlueRight":
                break;
            case "RedLeft":
                break;
            case "RedRight":
                break;



        }

         */

//        blinkinLedDriver = hwBot.get(RevBlinkinLedDriver.class, "Blinkin");
//        pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;
//        blinkinLedDriver.setPattern(pattern);



        // define motors for robot
        frontLeftMotor=hwBot.dcMotor.get("front_left_motor");
        frontRightMotor=hwBot.dcMotor.get("front_right_motor");
        rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");
        rearRightMotor = hwBot.dcMotor.get("rear_right_motor");
/*
        LauncherMotor = hwBot.dcMotor.get("launcher_motor");

        IntakeMotor = hwBot.dcMotor.get("intake_motor");


 */
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
/*
        LauncherMotor.setDirection(DcMotor.Direction.FORWARD);
//        LauncherMotor.setDirection(DcMotor.Direction.FORWARD);
//        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);


 */





        //Initialize Motor Run Mode for Robot
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        flag = hwBot.get(Servo.class,"flag");
//        flag.setDirection(Servo.Direction.FORWARD);

        // Define and Initialize Gyro
        BNO055IMU.Parameters parametersimu = new BNO055IMU.Parameters();
        parametersimu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersimu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersimu.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersimu.loggingEnabled = true;
        parametersimu.loggingTag = "IMU";
        parametersimu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwBot.get(BNO055IMU.class, "imu");
        imu.initialize(parametersimu);





    }

//    public void raiseFlag() {
//        flag.setPosition(0.5);
//    }
//    public void lowerFlag() {
//        flag.setPosition(0.9);
//    }
//
//    public void initFlag() {
//        flag.setPosition(0.2);
//    }
//    public void waveFlagRight() {
//        flag.setPosition(0.399);
//    }
//    public void waveFlagLeft() {
//        flag.setPosition(0.711);
//    }

    public void initCamera () {
//        int cameraMonitorViewId = hwBot.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwBot.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwBot.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        pipeline = new SkystoneDeterminationPipeline();
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                /*
//                 * Tell the webcam to start streaming images to us! Note that you must make sure
//                 * the resolution you specify is supported by the camera. If it is not, an exception
//                 * will be thrown.
//                 *
//                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
//                 * supports streaming from the webcam in the uncompressed YUV image format. This means
//                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
//                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
//                 *
//                 * Also, we specify the rotation that the webcam is used in. This is so that the image
//                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
//                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
//                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
//                 * away from the user.
//                 */
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//        });
    }

    /*
    public void servoClosed () {
        WobbleArm.setPosition(servoClosePos);
    }

    public void servoOpened(){
        WobbleArm.setPosition(servoOpenPos);
    }

    public void WobbleLower() {
        WobbleArm.setPosition(WobbleArmLowerPos);
    }
    public void WobbleRaised() {
        WobbleArm.setPosition(WobbleArmRaisedPos);
    }
    public void WobbleOpen(){
        WobbleGrab.setPosition(WobbleGrabOpenPos);
    }
    public void WobbleClosed(){
        WobbleGrab.setPosition(WobbleGrabClosePos);
    }

     */
    public void detectRings () { }

    /*

    public void LauncherOn(double power) {
        LauncherMotor.setPower(power);
    }
    public void LauncherOff(double power) {
        LauncherMotor.setPower(power);
    }

    public void IntakeOn(double power){
        IntakeMotor.setPower(power);
    }
    public void IntakeOff(double power){
        IntakeMotor.setPower(power);

    }

     */

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    ElapsedTime timer = new ElapsedTime();
    private double lastError;
    //    double encoderAverage = (leftMotorA.getCurrentPosition() + rightMotorA.getCurrentPosition())/2;
    double encoderAverage;

    public double PIDControl (double reference, double state){
        double error = reference + state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError)/timer.seconds();
        lastError = error;
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public void driveForwardPID (double rotations) {
        if (linearOp.opModeIsActive()) {
            double ticks = rotations * TICKS_PER_ROTATION;
            double speed = PIDControl(100, encoderAverage);     //100% power
            setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (encoderAverage < ticks && linearOp.opModeIsActive()) {
                driveForward(speed);
            }
        }
    }
    public void driveBackwardPID (double rotations) {
        if (linearOp.opModeIsActive()) {
            double ticks = rotations * TICKS_PER_ROTATION;
            double speed = PIDControl(100, encoderAverage);     //100% power
            setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (encoderAverage < ticks && linearOp.opModeIsActive()) {
                driveBackward(speed);
            }
        }
    }

    public void gyroCorrection (double speed, double angle) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (angles.firstAngle >= angle + TOLERANCE && linearOp.opModeIsActive()) {
            while (angles.firstAngle >=  angle + TOLERANCE && linearOp.opModeIsActive()) {
                rotateRight(speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                linearOp.telemetry.addData("Current Angle: ", angles.firstAngle);
            }
        }
        else if (angles.firstAngle <= angle - TOLERANCE && linearOp.opModeIsActive()) {
            while (angles.firstAngle <= angle - TOLERANCE && linearOp.opModeIsActive()) {
                rotateLeft(speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                linearOp.telemetry.addData("Current Angle: ", angles.firstAngle);
            }
        }
        stopMotors();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }




//    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
//    {
//
////          An enum to define the skystone position
//
//        public enum RingPosition
//        {
//            FOUR,
//            ONE,
//            NONE
//        }
//
//
////         Some color constants
//
//        static final Scalar BLUE = new Scalar(0, 0, 255);
//        static final Scalar GREEN = new Scalar(0, 255, 0);
//
//
////          The core values which define the location and size of the sample regions
//
//        //        USE THIS TO FIGURE OUT NUMBER OF RINGS.
//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);
//
//        //        ORIGINAL AREA
//        static final int REGION_WIDTH = 35;
//        static final int REGION_HEIGHT = 35;
//
//        //        ORIGINAL THRESHOLDS
//        final int FOUR_RING_THRESHOLD = 140;
//        final int ONE_RING_THRESHOLD = 127;
//
//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        /*
//         * Working variables
//         */
//        Mat region1_Cb;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        public int avg1;
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        public volatile EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition position = EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR;
//
//        /*
//         * This function takes the RGB frame, converts to YCrCb,
//         * and extracts the Cb channel to the 'Cb' variable
//         */
//        void inputToCb(Mat input)
//        {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 1);
//        }
//
//        @Override
//        public void init(Mat firstFrame)
//        {
//            inputToCb(firstFrame);
//
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//        }
//
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            inputToCb(input);
//
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//            position = EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
//            if(avg1 > FOUR_RING_THRESHOLD){
//                position = EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR;
//            }else if (avg1 > ONE_RING_THRESHOLD){
//                position = EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE;
//            }else{
//                position = EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE;
//            }
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    GREEN, // The color the rectangle is drawn in
//                    -1); // Negative thickness means solid fill
//
//            return input;
//        }
//
//        public int getAnalysis()
//        {
//            return avg1;
//        }
//    }



//    class SamplePipeline extends OpenCvPipeline {
//        boolean viewportPaused;
//
//        /*
//         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
//         * highly recommended to declare them here as instance variables and re-use them for
//         * each invocation of processFrame(), rather than declaring them as new local variables
//         * each time through processFrame(). This removes the danger of causing a memory leak
//         * by forgetting to call mat.release(), and it also reduces memory pressure by not
//         * constantly allocating and freeing large chunks of memory.
//         */
//
//        @Override
//        public Mat processFrame(Mat input) {
//            /*
//             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
//             * will only dereference to the same image for the duration of this particular
//             * invocation of this method. That is, if for some reason you'd like to save123 a copy
//             * of this particular frame for later use, you will need to either clone it or copy
//             * it to another Mat.
//             */
//
//            /*
//             * Draw a simple box around the middle 1/2 of the entire frame
//             */
//            Imgproc.rectangle(
//                    input,
//                    new Point(
//                            input.cols() / 4,
//                            input.rows() / 4),
//                    new Point(
//                            input.cols() * (3f / 4f),
//                            input.rows() * (3f / 4f)),
//                    new Scalar(0, 255, 0), 4);
//
//            /**
//             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
//             * to change which stage of the pipeline is rendered to the viewport when it is
//             * tapped, please see {@link PipelineStageSwitchingExample}
//             */
//
//            return input;
//        }
//
//        @Override
//        public void onViewportTapped() {
//            /*
//             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
//             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
//             * when you need your vision pipeline running, but do not require a live preview on the
//             * robot controller screen. For instance, this could be useful if you wish to see the live
//             * camera preview as you are initializing your robot, but you no longer require the live
//             * preview after you have finished your initialization process; pausing the viewport does
//             * not stop running your pipeline.
//             *
//             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
//             */
//
//            viewportPaused = !viewportPaused;
//
//            if (viewportPaused) {
//                webcam.pauseViewport();
//            } else {
//                webcam.resumeViewport();
//            }
//        }
//    }

    public void driveGyroBackward (double power, double rotations) throws InterruptedException {
        double ticks = rotations * (+1) * TICKS_PER_ROTATION;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPos = 0;
        double leftSideSpeed;
        double rightSideSpeed;


        double target = angles.firstAngle;
        double startPosition = frontLeftMotor.getCurrentPosition();
        linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
        linearOp.telemetry.update();
        linearOp.sleep(100);
//        while (currentPos < ticks + startPosition && linearOp.opModeIsActive()) {
        while (currentPos < ticks + startPosition && linearOp.opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());



            leftSideSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
            rightSideSpeed = power + (angles.firstAngle - target) / 100;

            leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
            rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);

            frontLeftMotor.setPower(-leftSideSpeed);
            rearLeftMotor.setPower(-leftSideSpeed);

            frontRightMotor.setPower(-rightSideSpeed);
            rearRightMotor.setPower(-rightSideSpeed);


//            switch (direction) {
//                case "forward":
//                    currentPos = frontLeftMotor.getCurrentPosition();
//                    leftSideSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
//                    rightSideSpeed = power - (angles.firstAngle - target) / 100;
//
//                    leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
//                    rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);
//
//                    frontLeftMotor.setPower(leftSideSpeed);
//                    rearLeftMotor.setPower(leftSideSpeed);
//
//                    frontRightMotor.setPower(rightSideSpeed);
//                    rearRightMotor.setPower(rightSideSpeed);
//                    break;
//                case "backward":
//                    currentPos = -frontLeftMotor.getCurrentPosition();
//                    leftSideSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
//                    rightSideSpeed = power + (angles.firstAngle - target) / 100;
//
//                    leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
//                    rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);
//
//                    frontLeftMotor.setPower(-leftSideSpeed);
//                    rearLeftMotor.setPower(-leftSideSpeed);
//
//                    frontRightMotor.setPower(-rightSideSpeed);
//                    rearRightMotor.setPower(-rightSideSpeed);
//                    break;
//            }
//
//
//
//
//
//            linearOp.telemetry.addData("Left Speed", frontLeftMotor.getPower());
//            linearOp.telemetry.addData("Right Speed", frontRightMotor.getPower());
//            linearOp.telemetry.addData("Distance till destination ", encoders + startPosition - frontLeftMotor.getCurrentPosition());
//            linearOp.telemetry.addData("Current Position", currentPos);
//            linearOp.telemetry.addData("Target Position", target);
//            linearOp.telemetry.addData("Angle: ", angles.firstAngle);
//            linearOp.telemetry.update();
            // missing waiting

            linearOp.idle();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        linearOp.idle();

    }
//matthew did nothing

    public void driveGyroForward (double power, double rotations) throws InterruptedException {

        double ticks = rotations * (1) * TICKS_PER_ROTATION;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPos = 0;
        double leftSideSpeed;
        double rightSideSpeed;


        double target = angles.firstAngle;
        double startPosition = frontLeftMotor.getCurrentPosition();
        //  linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
        //  linearOp.telemetry.update();
        linearOp.sleep(100);
        while (currentPos < ticks + startPosition && linearOp.opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());

            currentPos = frontLeftMotor.getCurrentPosition();
            leftSideSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
            rightSideSpeed = power - (angles.firstAngle - target) / 100;

            leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
            rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);

            frontLeftMotor.setPower(leftSideSpeed);
            rearLeftMotor.setPower(leftSideSpeed);

            frontRightMotor.setPower(rightSideSpeed);
            rearRightMotor.setPower(rightSideSpeed);



/*
            linearOp.telemetry.addData("Left Speed", frontLeftMotor.getPower());
            linearOp.telemetry.addData("Right Speed", frontRightMotor.getPower());
            linearOp.telemetry.addData("Distance till destination ", encoders + startPosition - frontLeftMotor.getCurrentPosition());
            linearOp.telemetry.addData("Current Position", currentPos);
            linearOp.telemetry.addData("Target Position", target);
            linearOp.telemetry.addData("Angle: ", angles.firstAngle);
            linearOp.telemetry.update();
            // missing waiting
*/
            linearOp.idle();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        linearOp.idle();


        /*
//        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double ticks = rotations * (1) * TICKS_PER_ROTATION;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPos = 0;
        double leftSideSpeed;
        double rightSideSpeed;


        double target = angles.firstAngle;
        double startPosition = frontLeftMotor.getCurrentPosition();
          linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
          linearOp.telemetry.update();
        linearOp.sleep(100);
        while (currentPos < ticks + startPosition && linearOp.opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());



            leftSideSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
            rightSideSpeed = power + (angles.firstAngle - target) / 100;

            leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
            rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);

            frontLeftMotor.setPower(+leftSideSpeed);
            rearLeftMotor.setPower(+leftSideSpeed);

            frontRightMotor.setPower(+rightSideSpeed);
            rearRightMotor.setPower(+rightSideSpeed);


//            switch (direction) {
//                case "forward":
//                    currentPos = frontLeftMotor.getCurrentPosition();
//                    leftSideSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
//                    rightSideSpeed = power - (angles.firstAngle - target) / 100;
//
//                    leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
//                    rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);
//
//                    frontLeftMotor.setPower(leftSideSpeed);
//                    rearLeftMotor.setPower(leftSideSpeed);
//
//                    frontRightMotor.setPower(rightSideSpeed);
//                    rearRightMotor.setPower(rightSideSpeed);
//                    break;
//                case "backward":
//                    currentPos = -frontLeftMotor.getCurrentPosition();
//                    leftSideSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
//                    rightSideSpeed = power + (angles.firstAngle - target) / 100;
//
//                    leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
//                    rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);
//
//                    frontLeftMotor.setPower(-leftSideSpeed);
//                    rearLeftMotor.setPower(-leftSideSpeed);
//
//                    frontRightMotor.setPower(-rightSideSpeed);
//                    rearRightMotor.setPower(-rightSideSpeed);
//                    break
//            }
//
//
//
//
//
//            linearOp.telemetry.addData("Left Speed", frontLeftMotor.getPower());
//            linearOp.telemetry.addData("Right Speed", frontRightMotor.getPower());
//            linearOp.telemetry.addData("Distance till destination ", ticks + startPosition - frontLeftMotor.getCurrentPosition());
//            linearOp.telemetry.addData("Current Position", currentPos);
//            linearOp.telemetry.addData("Target Position", target);
//            linearOp.telemetry.addData("Angle: ", angles.firstAngle);
//            linearOp.telemetry.update();
            // missing waiting

            linearOp.idle();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        linearOp.idle();
*/
    }


    public void driveGyroStraight (int encoders, double power) throws InterruptedException {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPos = 0;
        double leftSideSpeed;
        double rightSideSpeed;


        double target = angles.firstAngle;
        double startPosition = frontLeftMotor.getCurrentPosition();
        //  linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
        //  linearOp.telemetry.update();
        linearOp.sleep(100);
        while (currentPos < encoders + startPosition && linearOp.opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());

            currentPos = frontLeftMotor.getCurrentPosition();
            leftSideSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
            rightSideSpeed = power - (angles.firstAngle - target) / 100;

            leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
            rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);

            frontLeftMotor.setPower(leftSideSpeed);
            rearLeftMotor.setPower(leftSideSpeed);

            frontRightMotor.setPower(rightSideSpeed);
            rearRightMotor.setPower(rightSideSpeed);



/*
            linearOp.telemetry.addData("Left Speed", frontLeftMotor.getPower());
            linearOp.telemetry.addData("Right Speed", frontRightMotor.getPower());
            linearOp.telemetry.addData("Distance till destination ", encoders + startPosition - frontLeftMotor.getCurrentPosition());
            linearOp.telemetry.addData("Current Position", currentPos);
            linearOp.telemetry.addData("Target Position", target);
            linearOp.telemetry.addData("Angle: ", angles.firstAngle);
            linearOp.telemetry.update();
            // missing waiting
*/
            linearOp.idle();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        linearOp.idle();
//
    }

    public void driveGyroStrafe (double power, double rotations, String direction) throws InterruptedException {
        double ticks = 0;
        ticks = rotations * TICKS_PER_ROTATION;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPos = 0;
        double frontLeftSpeed;
        double frontRightSpeed;
        double rearLeftSpeed;
        double rearRightSpeed;


        double target = angles.firstAngle;
        double startPosition = frontLeftMotor.getCurrentPosition();
        linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
        linearOp.telemetry.update();
        linearOp.sleep(2000);
        while (currentPos < ticks + startPosition && linearOp.opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());

            switch (direction) {
                case "left":
                    frontLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
                    frontRightSpeed = power - (angles.firstAngle - target) / 100;
                    rearLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
                    rearRightSpeed = power + (angles.firstAngle - target) / 100;

                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                    frontLeftMotor.setPower(-frontLeftSpeed);
                    frontRightMotor.setPower(frontRightSpeed);

                    rearLeftMotor.setPower(rearLeftSpeed);
                    rearRightMotor.setPower(-rearRightSpeed);
                    break;
                case "right":
                    frontLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
                    frontRightSpeed = power + (angles.firstAngle - target) / 100;
                    rearLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
                    rearRightSpeed = power - (angles.firstAngle - target) / 100;

                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                    frontLeftMotor.setPower(frontLeftSpeed);
                    frontRightMotor.setPower(-frontRightSpeed);

                    rearLeftMotor.setPower(-rearLeftSpeed);
                    rearRightMotor.setPower(rearRightSpeed);
                    break;
            }



//            linearOp.telemetry.addData("Left Speed", frontLeftMotor.getPower());
//            linearOp.telemetry.addData("Right Speed", frontRightMotor.getPower());
//            linearOp.telemetry.addData("Distance till destination ", encoders + startPosition - frontLeftMotor.getCurrentPosition());
//            linearOp.telemetry.addData("Current Position", currentPos);
//            linearOp.telemetry.addData("Target Position", target);
//            linearOp.telemetry.addData("Angle: ", angles.firstAngle);
//
//            linearOp.telemetry.update();

            // missing waiting
            linearOp.idle();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        linearOp.idle();

    }



    public void driveGyroStrafeAngle (double power, double rotations, String direction, double angle) throws InterruptedException {
        double ticks = 0;
        ticks = rotations * TICKS_PER_ROTATION;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPos = 0;
        double frontLeftSpeed;
        double frontRightSpeed;
        double rearLeftSpeed;
        double rearRightSpeed;


        double target = angle;
        double startPosition = frontLeftMotor.getCurrentPosition();
        linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
        linearOp.telemetry.update();
        linearOp.sleep(2000);
        while (currentPos < ticks + startPosition && linearOp.opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());

            switch (direction) {
                case "left":
                    frontLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
                    frontRightSpeed = power - (angles.firstAngle - target) / 100;
                    rearLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
                    rearRightSpeed = power + (angles.firstAngle - target) / 100;

                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                    frontLeftMotor.setPower(-frontLeftSpeed);
                    frontRightMotor.setPower(frontRightSpeed);

                    rearLeftMotor.setPower(rearLeftSpeed);
                    rearRightMotor.setPower(-rearRightSpeed);
                    break;
                case "right":
                    frontLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
                    frontRightSpeed = power + (angles.firstAngle - target) / 100;
                    rearLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
                    rearRightSpeed = power - (angles.firstAngle - target) / 100;

                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                    frontLeftMotor.setPower(frontLeftSpeed);
                    frontRightMotor.setPower(-frontRightSpeed);

                    rearLeftMotor.setPower(-rearLeftSpeed);
                    rearRightMotor.setPower(rearRightSpeed);
                    break;
            }



//           linearOp.telemetry.addData("Left Speed", frontLeftMotor.getPower());
//           linearOp.telemetry.addData("Right Speed", frontRightMotor.getPower());
//           linearOp.telemetry.addData("Distance till destination ", encoders + startPosition - frontLeftMotor.getCurrentPosition());
//           linearOp.telemetry.addData("Current Position", currentPos);
//           linearOp.telemetry.addData("Target Position", target);
//           linearOp.telemetry.addData("Angle: ", angles.firstAngle);
//
//           linearOp.telemetry.update();

            // missing waiting
            linearOp.idle();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        linearOp.idle();



    }

    public void driveForward_PIDRTP (double power, double rotations) {
        int ticks = (int) (rotations * TICKS_PER_ROTATION);

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



//        frontLeftMotor.setPower(power);
//        frontLeftMotor.setTargetPosition(ticks);

        frontRightMotor.setPower(power);
        frontRightMotor.setTargetPosition(ticks);

//        rearLeftMotor.setPower(power);
//        rearLeftMotor.setTargetPosition(ticks);
//
//        rearRightMotor.setPower(power);
//        rearRightMotor.setTargetPosition(ticks);

        setMotorRunModes(DcMotor.RunMode.RUN_TO_POSITION);
        while (frontLeftMotor.isBusy() && linearOp.opModeIsActive()) {
            linearOp.idle();
        }
        stopMotors();

//        linearOp.sleep(500);
    }

}


