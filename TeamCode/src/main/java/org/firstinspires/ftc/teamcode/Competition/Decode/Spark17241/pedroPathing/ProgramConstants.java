package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ProgramConstants {


    // Weight of Robot
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.0)   // Kilograms of Robot Weighed on 1/17 oz
            .forwardZeroPowerAcceleration(-37.186)    // Tested on 1/19 (Andrew)
            .lateralZeroPowerAcceleration(-59.037);   // Tested on 1/19 (Andrew)

    // Drivetrain Constants based on DecBot
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_motor")
            .rightRearMotorName("rear_right_motor")
            .leftRearMotorName("rear_left_motor")
            .leftFrontMotorName("front_left_motor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(61.148)    // Tested on 1/19 (Andrew)
            .yVelocity(51.230);   // Tested on 1/19 (Andrew)

    // Localization (Pinpoint Two Wheel Odometry Constants
    // Use the Qualcom GoBiilda Drive, not local package

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)         // Measured on 1/19 (Andrew)
            .strafePodX(-2)        // Measured on 1/19 (Andrew)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)  //
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)  // Tested 10/28
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);  // Tested 10/28

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)   // Fast Constants
                .build();
    }

    // Slow Drive Constants and Slow Follower

    public static MecanumConstants slowDriveConstants = new MecanumConstants()
            .maxPower(.25)
            .rightFrontMotorName("front_right_motor")
            .rightRearMotorName("rear_right_motor")
            .leftRearMotorName("rear_left_motor")
            .leftFrontMotorName("front_left_motor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(50.78)    // Tested on 1/17
            .yVelocity(58.3140);   // Tested on 1/17

    public static Follower slowFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(slowDriveConstants)    // Slow Constants
                .build();
    }

}