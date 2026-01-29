package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Robots.StateDecodeBot;

public abstract class AutoMainNew extends OpMode {

    protected final ElapsedTime opmodeTimer = new ElapsedTime();

    protected double target_velocity = 0.0;
    protected double gateTolerance = 20.0;

    protected int shotsFired = 0;

    public StateDecodeBot decBot = new StateDecodeBot();

    // ----------------- NORMAL POSES ---------------------------

    //------------Blue----------------
    public final Pose BlueFarStartPose = new Pose(44, 8, Math.toRadians(90));
    public final Pose BlueMidShootPose = new Pose(59, 81, Math.toRadians(133));
    public final Pose BlueFarShootPose = new Pose(60, 20, Math.toRadians(110));
    public final Pose BlueFarParkPose  = new Pose(43, 12, Math.toRadians(0));

    public final Pose BlueSpikeAInsidePose  = new Pose(48, 36, Math.toRadians(180));
    public final Pose BlueSpikeAOutsidePose = new Pose(16, 36, Math.toRadians(180));

    public final Pose BlueSpikeBInsidePose  = new Pose(48, 61, Math.toRadians(180));
    public final Pose BlueSpikeBOutsidePose = new Pose(16, 61, Math.toRadians(180));

    public final Pose BlueSpikeCInsidePose  = new Pose(97, 85, Math.toRadians(0));
    public final Pose BlueSpikeCOutsidePose = new Pose(128, 85, Math.toRadians(0));

    //------------Red-----------------
    public final Pose RedFarStartPose =  new Pose(100, 8, Math.toRadians(90));

    public final Pose RedMidShootPose = new Pose(85, 81, Math.toRadians(45));
    public final Pose RedFarShootPose = new Pose(84, 20, Math.toRadians(64));
    public final Pose RedFarParkPose  = new Pose(101, 12, Math.toRadians(90));

    public final Pose RedSpikeAInsidePose  = new Pose(48, 36, Math.toRadians(180));
    public final Pose RedSpikeAOutsidePose = new Pose(16, 36, Math.toRadians(180));

    public final Pose RedSpikeBInsidePose  = new Pose(48, 61, Math.toRadians(180));
    public final Pose RedSpikeBOutsidePose = new Pose(16, 61, Math.toRadians(180));

    public final Pose RedSpikeCInsidePose  = new Pose(48, 85, Math.toRadians(180));
    public final Pose RedSpikeCOutsidePose = new Pose(16, 85, Math.toRadians(180));


    public double getCurrentVelocity() {
        return (decBot.launchFrontMotor.getVelocity() + decBot.launchBackMotor.getVelocity()) / 2.0;
    }
    public boolean LaunchWheelsInGate(double selected_speed, double tolerance) {

        // Correct version (keep for real robot):
        // double upper = Math.max(0, selected_speed + tolerance);
        // double lower = Math.max(0, selected_speed - tolerance);
        // double current = getCurrentVelocity();
        // return (current >= lower && current <= upper);
        return true;
    }

    public void prepareForLaunch() {
        // Keep flywheel commanded to target
        decBot.flylaunch(target_velocity);
    }

    // Burner launch (spin wheels after 5 seconds from startTime)
    // Returns true once it has begun spinning
    public boolean burnerLaunch(double velocity, double currentTimeMs, double startTimeMs) {
        if ((currentTimeMs - startTimeMs) >= 5000) {
            decBot.flylaunch(velocity);
            return true;
        }
        return false;
    }


    protected float fireDelay = 5.0f * 1000;
    protected double launchStartTimeMs = 0;
    protected boolean hasStartedPrepare = false;
    protected boolean hasStartedLaunch = false;

    // Returns true when ONE firing cycle is done
    public boolean LaunchBalls(double target) {
        target_velocity = target;
        decBot.flylaunch(target_velocity);

        if (!hasStartedPrepare) {
            hasStartedPrepare = true;
            prepareForLaunch();
        }

        if (LaunchWheelsInGate(target_velocity, gateTolerance) && !hasStartedLaunch) {
            hasStartedLaunch = true;
            launchStartTimeMs = opmodeTimer.milliseconds();
            decBot.beginFeed();
        }

        if (hasStartedLaunch && (opmodeTimer.milliseconds() - launchStartTimeMs >= fireDelay)) {
            shotsFired++;

            hasStartedPrepare = false;
            hasStartedLaunch = false;

            decBot.stopFeed();
            decBot.flylaunch(0);
            return true;
        }

        return false;
    }

    public void AutoMainTelemetry() {
        telemetry.addData("target velocity", target_velocity);
        telemetry.addData("flywheel speed", getCurrentVelocity());
        telemetry.addData("Gate Tolerance", gateTolerance);
        telemetry.addData("InGate", LaunchWheelsInGate(target_velocity, gateTolerance));
        telemetry.addData("shots fired", shotsFired);
        telemetry.addData("hasStartedLaunch", hasStartedLaunch);
    }
}
