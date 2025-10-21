package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Pinpoint {
    public HardwareMap hwBot = null;
    public PinpointDriver pinpoint = null;
    public LinearOpMode LinearOp = null;

    // This is the amount of MM from the pinpoint computer hardware as compared to the logical center of the robot
    public double x = 101.6;    // Need to update these for Blue Program Bot -53.975
    public double y = -38.1;   // Need to update these for Blue Program Bot  88.9

    public Pinpoint() {
    }

    public void setLinearOp(LinearOpMode LinearOp) {
        this.LinearOp = LinearOp;
    }

    public void initPinpoint(HardwareMap hwMap) {
        hwBot = hwMap;

        pinpoint = hwBot.get(PinpointDriver.class, "odo");
        pinpoint.setOffsets(x, y);
        pinpoint.setEncoderResolution(PinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(PinpointDriver.EncoderDirection.REVERSED, PinpointDriver.EncoderDirection.FORWARD);

        // resets the position and IMU heading to their default state
        pinpoint.resetPosAndIMU();

    }

    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }

    public Pose2D getPosition() {
        return pinpoint.getPosition();

    }

    public Pose2D getVelocity() {
        return pinpoint.getVelocity();

    }

    public void updatePosition(Pose2D position) {
        pinpoint.setPosition(position);
    }

    public void updateXYPosition(double x, double y) {
        Pose2D oldPosition = getPosition();
        double yaw = oldPosition.getHeading(AngleUnit.DEGREES);

        Pose2D newPosition = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, yaw);
        updatePosition(newPosition);
    }

    public void updateHeading(double heading) {
        Pose2D oldPosition = getPosition();
        double x = oldPosition.getX(DistanceUnit.INCH);
        double y = oldPosition.getY(DistanceUnit.INCH);

        Pose2D newPosition = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
        updatePosition(newPosition);
    }
}
