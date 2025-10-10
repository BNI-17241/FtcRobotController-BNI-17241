package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots;

//import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Modules.EasyOpenCVWebcam;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


public class CompetitionBot extends MecanumDrive {

    //hardware constructors
    public HardwareMap hwBot  =  null;


    public VoltageSensor voltageSensor = null;

    public double launchCoefficient;

//GYRO INITIALIZATION

    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public final double SPEED = .3;
    public final double TOLERANCE = .4;

    // Color and Distance Hardware & Variables
//    public ColorSensor sensorColorWobbleArm;
//    public DistanceSensor sensorDistanceWobbleArm;
    public float hsvValues[] = {0F, 0F, 0F};
    public final double SCALE_FACTOR = 1;
//    Under 100 = red tape
    public static final int WOBBLE_ARM_RAISE_THRESHOLD = 170;
//    Looks for > 200 with blue tape
    public static final int WOBBLE_ARM_LOWER_THRESHOLD = 100;
    //  Camera Initialization



    public OpenCvCamera webcam;
    public SkystoneDeterminationPipeline pipeline;

//    public static final double TICKS_PER_ROTATION = 383.6;   // GoBilda 13.7 Motor PPR




//    Servo WobbleArm = null;
    Servo WobbleGrab = null;
    Servo Camera = null;
    public Servo ServoRingPusher = null;
    public Servo RingMag = null;
    public CRServo IntakeCorrector = null;

//    Servo WobbleArmStop = null;
//    Servos servos = new Servos();


    public double servoOpenPos = 0.36;
    public double servoClosePos = 0.93;
//    was 0.446
//    public double WobbleArmRaisedPos = 0.23;
//    public double WobbleArmLowerPos = 0.613;
    public double WobbleGrabOpenPos = .59;
    public double WobbleGrabClosePos = 0.116;
    //Blue Left:
    //was at .2
    public double CameraServoPosBlueLeft = 0.2;
    //Blue Right:
    public double CameraServoPosBlueRight = 0.602;
    //Launcher Motor:
    public DcMotor IntakeMotor = null;
    public double RingPushPos = 0.26;
    //.196 before (Mar 3, 2021 3:40pm)
    public double RingPullPos = 0.40;
    //.460 before (Mar 3, 2021 3:40pm)
    public double RingMagUpPos = 0.2;
    //.13 before (Mar 3, 2021 3:42pm)
    public double RingMagDownPos = 0.051;
    //.058 before (Mar 3, 2021 3:42pm)
    public double RingMagUpAuto = 0.217;

    public double DeltaRing = Math.abs(RingPullPos - RingPushPos);
    int numLoops = 7;
    public double ringIncrement = DeltaRing/numLoops;

    public double DeltaRingMag = Math.abs(RingMagUpPos - RingMagDownPos);
    int numLoopsMag = 9;
    //if numloops is high = servo is slower
    //if numloops is low = servo is faster
    public double ringIncrementMag = DeltaRingMag/numLoopsMag;

//    public double WobbleArmStopOpen = 0.02;
//    public double WobbleArmStopClose = 0.48;
//    Wobble Arm Motor Data

    public DcMotor WobbleArmMotor = null;
//    public DcMotor motor_left = null;
//    public DcMotor motor_right = null;
    public DcMotorEx launcherMotor1 = null;
    public DcMotorEx launcherMotor2 = null;
    public double velocity = 1600;
    public boolean wobbleArmRaiseEngage;
    public boolean wobbleArmLowerengage;

    public double cameraInitPos = 0.4;
    public double cameraDetectPos = 0.55;
//.78 before

    public int rapidFireRing = 0;
    public int rapidPushTimer = 150; //200 before
    public int rapidPullTimer = 175; //150 before
    public boolean launchModePush = false;
    public boolean launchModePull = false;

    private final static int LED_PERIOD = 10;
    private final static int GAMEPAD_LOCKOUT = 500;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;

    public double maxWobbleArmRaiseTime = 0.3;
    public double maxWobbleArmLowerTime = 2; //2.0 before
    public double maxRingPusherTime = 0.3;
    public double maxRingPullerTime = 0.3;

    public ElapsedTime wobbleArmTimer;
    public ElapsedTime ringPusherTimer;



    Telemetry.Item patternName;
    Telemetry.Item display;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }
    //LabBot constructor
    public CompetitionBot() {

    }
    public void CameraDetect(){
        Camera.setPosition(cameraDetectPos);
    }

    public void CameraInit(){
        Camera.setPosition(cameraInitPos);
    }

    public void initRobot(HardwareMap hardwareMap, String startPosition, String mode){
//        HardwareMap hwMap, String startPosition, String mode

//        hwBot = hwMap
        hwBot = hardwareMap;
//        WobbleArm = hwBot.get(Servo.class, "wobble_arm");
//        WobbleArm.setDirection(Servo.Direction.FORWARD);
//        if (mode.equals("auto")) {
//            WobbleArm.setPosition(WobbleArmRaisedPos);
//
//        }


        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launchCoefficient = 12 / voltageSensor.getVoltage();

        WobbleGrab = hwBot.get(Servo.class, "wobble_grab");
        WobbleGrab.setDirection(Servo.Direction.FORWARD);
        RingMag = hwBot.get(Servo.class, "ring_mag");
        ServoRingPusher = hwBot.get(Servo.class, "servo_ring_pusher");
//        WobbleArmStop = hwBot.get(Servo.class, "wobble_arm_stopper");
        Camera = hwBot.get(Servo.class, "camera_servo");
//        Camera.setDirection(Servo.Direction.FORWARD);
        if (mode.equals("auto")){
            WobbleGrab.setPosition(WobbleGrabClosePos);
//            WobbleArmStopOpen();
            RingMagUp();
            RingPush();

            Camera.setPosition(CameraServoPosBlueLeft);
        }
        Camera = hwBot.get(Servo.class, "camera_servo");
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

        blinkinLedDriver = hwBot.get(RevBlinkinLedDriver.class, "Blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;
        blinkinLedDriver.setPattern(pattern);

//        IntakeCorrector = hwBot.get(CRServo.class, "intake_corrector");
        IntakeCorrector = hardwareMap.crservo.get ("intake_corrector");
        IntakeCorrector.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeCorrector.setPower(0);



        // define motors for robot
        frontLeftMotor=hwBot.dcMotor.get("front_left_motor");
        frontRightMotor=hwBot.dcMotor.get("front_right_motor");
        rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");
        rearRightMotor = hwBot.dcMotor.get("rear_right_motor");


        IntakeMotor = hwBot.dcMotor.get("intake_motor");

        WobbleArmMotor = hwBot.dcMotor.get("wobble_arm_motor");


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);


//        motor_left = hwMap.dcMotor.get("launcher_motor_l");
//        motor_left.setDirection(DcMotorSimple.Direction.FORWARD);
//        motor_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor_right = hwMap.dcMotor.get("launcher_motor_r");
//        motor_right.setDirection(DcMotorSimple.Direction.FORWARD);
//        motor_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



//        launcherMotor1 = hwMap.dcMotor.get("launcher_motor_1");
        launcherMotor1 = hwBot.get(DcMotorEx.class, "launcher_motor_1");
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        launcherMotor2 = hwMap.dcMotor.get("launcher_motor_2");
        launcherMotor2 = hwBot.get(DcMotorEx.class, "launcher_motor_2");
        launcherMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        LauncherMotor.setDirection(DcMotor.Direction.FORWARD);
//        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        WobbleArmMotor.setDirection(DcMotor.Direction.FORWARD);
        WobbleArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        wobbleArmRaiseEngage = false;
        wobbleArmLowerengage = false;


        //Initialize Motor Run Mode for Robot
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        Color distance sensor

//        sensorColorWobbleArm = hwBot.get(ColorSensor.class, "sensor_color_distance_wobblearm");
//        sensorDistanceWobbleArm = hwBot.get(DistanceSensor.class, "sensor_color_distance_wobblearm");

//      Timers
        wobbleArmTimer = new ElapsedTime();
        wobbleArmTimer.reset();

        ringPusherTimer = new ElapsedTime();
        ringPusherTimer.reset();



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

    public void initCamera () {
//        int cameraMonitorViewId = hwBot.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwBot.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwBot.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

//    public void servoClosed () {
//        WobbleArm.setPosition(servoClosePos);
//    }
//
//    public void servoOpened(){
//        WobbleArm.setPosition(servoOpenPos);
//    }
//
//    public void WobbleLower() {
//        WobbleArm.setPosition(WobbleArmLowerPos);
//    }
//    public void WobbleRaised() {
//        WobbleArm.setPosition(WobbleArmRaisedPos);
//    }
    public void WobbleOpen(){
        WobbleGrab.setPosition(WobbleGrabOpenPos);
    }
    public void WobbleClosed(){
        WobbleGrab.setPosition(WobbleGrabClosePos);
    }
    public void detectRings () { }

    public void LauncherOn(double power) {
        launcherMotor1.setVelocity(power);
        launcherMotor2.setVelocity(power);
     }
    public void LauncherOff(double power) {
        launcherMotor1.setPower(power);
        launcherMotor2.setPower(power);
    }

    public void IntakeOn(double power){
        IntakeMotor.setPower(power);
    }
    public void IntakeOff(double power){
        IntakeMotor.setPower(power);
    }
    public void WobbleArmRaised(double power){WobbleArmMotor.setPower(power);}
    public void WobbleArmLower(double power){WobbleArmMotor.setPower(-power);}



//    sensorWobbleArmLower() == false &&
    public void WobbleArmLowerColorSensor () {
        wobbleArmTimer.reset();
        while (linearOp.opModeIsActive() && wobbleArmTimer.time() < maxWobbleArmLowerTime) {
            WobbleArmLower(1.0);
        }
        WobbleArmStopMotors();
    }
    public void RingPusherTimer(){
        ringPusherTimer.reset();
        while (linearOp.opModeIsActive() && ringPusherTimer.time() < maxRingPusherTime){
            RingPush();
        }
    }
    public void RingPullerTimer(){
        while (linearOp.opModeIsActive() && ringPusherTimer.time() < maxRingPullerTime){
            RingPull();
        }
    }

    public void RingFullAuto(){
//        sleeeeeeeeeeaaaaaspoprts its in the game3 = autonomousmonkey
//        what are these random words?
    }

    public void SpinInIntakeCorrector(){
        IntakeCorrector.setPower(1);
    }

//    public void SpinOutIntakeCorrector(){
//        IntakeCorrector.setPower(-1);
//    }

    public void StopIntakeCorrector(){
        IntakeCorrector.setPower(0);
    }

    public void FirstLaunch(){
        RingPull();
        ringPusherTimer.reset();
        launchModePush = true;
        launchModePull = false;
        rapidFireRing++;
//        linearOp.telemetry.addLine("FIRST");
//        linearOp.telemetry.update();
    }
    public void rapidFire(){
        if (rapidFireRing == 0){
            ringPusherTimer.reset();
            rapidFireRing++;
            launchModePush = true;
            launchModePull = false;
        }
        else if (rapidFireRing == 1){
            FirstLaunch();
        }
        else if (rapidFireRing == 2){
            rapidFireOp();
        }
        else if (rapidFireRing == 3){
            rapidFireOp();
        }
        else if (rapidFireRing == 4){
            rapidFireOp();

        }
        else if (rapidFireRing == 5){
            rapidFireOp();
        }
        else if (rapidFireRing == 6){
            rapidFireOp();
        }
        else{
            RingPull();
        }

    }

    public void rapidFireOp(){
        if (ringPusherTimer.milliseconds() > rapidPushTimer && launchModePush == true){
            RingPush();
            ringPusherTimer.reset();
            launchModePush = false;
            launchModePull = true;
        }
        else if (ringPusherTimer.milliseconds() > rapidPullTimer && launchModePull == true){
            RingPull();
            ringPusherTimer.reset();
            rapidFireRing++;
            launchModePush = true;
            launchModePull = false;
        }
    }
    public void RingPusherPullBack(){
    }

    public void WobbleArmRaiseColorSensor () {
        while (linearOp.opModeIsActive()  && wobbleArmTimer.time() < maxWobbleArmRaiseTime) {
            WobbleArmRaised(0.8);
        }
        WobbleArmStopMotors();
    }

    public void WobbleArmStopMotors () {
        WobbleArmMotor.setPower(0);
    }
    //        Pulls back pusher

    public void RingPush() {
        ServoRingPusher.setPosition(RingPushPos);
    }
    //        Pushes ring to launch!  Zoom Zoom
    public void RingPull() {
        ServoRingPusher.setPosition(RingPullPos);
    }

    public void RingPullIncrement(){
        if (ServoRingPusher.getPosition() <= RingPullPos) {
            ServoRingPusher.setPosition(ServoRingPusher.getPosition() + ringIncrement);
        }
    }
    public void RingMagIncrement(){
        if (RingMag.getPosition() <= RingMagUpPos){
            RingMag.setPosition(RingMag.getPosition() + ringIncrementMag);
        }
    }
    public void RingMagUp(){
        RingMag.setPosition(RingMagUpAuto);
    }
    public void RingMagDown(){
        RingMag.setPosition(RingMagDownPos);
    }
//    public void WobbleArmStopClose() {
//        WobbleArmStop.setPosition(WobbleArmStopClose);
//    }
//    public void WobbleArmStopOpen() {
//        WobbleArmStop.setPosition(WobbleArmStopOpen);
//    }
//    public boolean sensorWobbleArmRaise () {
//        Color.RGBToHSV((int) (sensorColorWobbleArm.red() * SCALE_FACTOR),
//                (int) (sensorColorWobbleArm.green() * SCALE_FACTOR),
//                (int) (sensorColorWobbleArm.blue() * SCALE_FACTOR),
//                hsvValues);
//
////WOBBLE_ARM_RAISE_THRESHOLD = 200
//        if (hsvValues[0] < WOBBLE_ARM_RAISE_THRESHOLD) {
//            return false;
//        }
//        else {
//            return true;
//        }
//    }
//
//    public boolean sensorWobbleArmLower () {
//        Color.RGBToHSV((int) (sensorColorWobbleArm.red() * SCALE_FACTOR),
//                (int) (sensorColorWobbleArm.green() * SCALE_FACTOR),
//                (int) (sensorColorWobbleArm.blue() * SCALE_FACTOR),
//                hsvValues);
////        linearOp.telemetry.addData("function hue ", hsvValues[0]);
////        linearOp.telemetry.update();
////        WOBBLE_ARM_LOWER_THRESHOLD = 50
//        if (hsvValues[0] > WOBBLE_ARM_LOWER_THRESHOLD) {
//            return false;
//        }
//        else {
//            return true;
//        }
//    }


    public void gyroCorrection (double speed, double angle) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        linearOp.telemetry.addData("current angle to start: ", angles.firstAngle);
        linearOp.telemetry.update();
//        linearOp.sleep(2000);
//        linearOp.sleep(1000);
        if (angles.firstAngle >= angle + TOLERANCE && linearOp.opModeIsActive()) {
            while (angles.firstAngle >=  angle + TOLERANCE && linearOp.opModeIsActive()) {
                rotateRight(speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                linearOp.telemetry.addData("current angle > : ", angles.firstAngle);
                linearOp.telemetry.update();
            }
        }
        else if (angles.firstAngle <= angle - TOLERANCE && linearOp.opModeIsActive()) {
            while (angles.firstAngle <= angle - TOLERANCE && linearOp.opModeIsActive()) {
                rotateLeft(speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                linearOp.telemetry.addData("current angle < : ", angles.firstAngle);
                linearOp.telemetry.update();
            }
        }
        stopMotors();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }



    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */

        //        USE THIS TO FIGURE OUT NUMBER OF RINGS.
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        //        ORIGINAL AREA
        static final int REGION_WIDTH = 35;
//        Was 35 for detecing from wall - this if for detecting from launch.  - 1/12/21 @ noon
        static final int REGION_HEIGHT = 100;

        //        ORIGINAL THRESHOLDS
        final int FOUR_RING_THRESHOLD = 145;
        final int ONE_RING_THRESHOLD = 127;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        public int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition position = EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = EasyOpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input) {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save123 a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped() {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }

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
//        linearOp.sleep(2000);
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




}
