package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.AprilTags;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

/**
 * A simple OpMode for testing AprilTag detection.
 */
@TeleOp(name = "AprilTagTest", group = "Testing")
public class AprilTagTest extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        initVision();

        telemetry.addData("Status", "Initialized. Ready to detect AprilTags.");
        telemetry.update();

        // Wait for the start command from the Driver Station.
        waitForStart();

        while (opModeIsActive()) {

            // Display tag information in telemetry.
            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Share the CPU.
            sleep(20);
        }

        // Clean up the vision portal when the OpMode stops.
        visionPortal.close();
    }

    /**
     * Initializes the vision system.
     */
    private void initVision() {

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(com.qualcomm.robotcore.hardware.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Display AprilTag information in the telemetry.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ (in): %6.1f %6.1f %6.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY (deg): %6.1f %6.1f %6.1f", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE (rad): %6.1f %6.1f %6.1f", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center (px): %6.0f %6.0f", detection.center.x, detection.center.y));
            }
        }
    }
}
