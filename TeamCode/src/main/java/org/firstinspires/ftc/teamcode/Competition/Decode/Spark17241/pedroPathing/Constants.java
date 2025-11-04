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

public class Constants {

    // Weight of Robot
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.2)   // Kilograms of Robot Weighed on 10/28
            .forwardZeroPowerAcceleration(-33.8697)    // Tested on 10/30
            .lateralZeroPowerAcceleration(-65.6387);   // Tested on 10/30

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
            .xVelocity(61.8028)    // Tested on 10/30
            .yVelocity(45.3907);   // Tested on 10/30

    // Localization (Pinpoint Two Wheel Odometry Constants
    // Use the Qualcom GoBiilda Drive, not local package

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1)         // Measured on 10/28
            .strafePodX(0.0)        // Measured on 10/28
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)  //
            .forwardEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED)  // Tested 10/28
            .strafeEncoderDirection(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED);  // Tested 10/28

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
            .xVelocity(61.8028)    // Tested on 10/30
            .yVelocity(45.3907);   // Tested on 10/30

    public static Follower slowFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(slowDriveConstants)    // Slow Constants
                .build();
    }

}