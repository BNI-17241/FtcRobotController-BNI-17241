package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Workspaces.Acker;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Disabled
@TeleOp(name="LL Distance Demo")
public class LimelightTester extends OpMode {

    private Limelight3A ll;

    @Override public void init() {
        ll = hardwareMap.get(Limelight3A.class, "limelight"); // match your config name
        ll.pipelineSwitch(0);           // your AprilTag pipeline #
        ll.setPollRateHz(50);           // how often to poll (Hz)
        ll.start();                     // begin streaming
    }

    @Override public void loop() {
        LLResult r = ll.getLatestResult();
        if (r != null && r.isValid() && !r.getFiducialResults().isEmpty()) {
            LLResultTypes.FiducialResult f = r.getFiducialResults().get(0);   // best tag this frame
            Pose3D tagInCam = f.getTargetPoseCameraSpace();     // pose of tag in CAMERA space
            double x = tagInCam.getPosition().x;  // right (+)
            double y = tagInCam.getPosition().y;  // down (+)
            double z = tagInCam.getPosition().z;  // forward/out of camera (+)

            double forwardMeters = z;                               // straight-ahead distance
            double rangeMeters = Math.sqrt(x*x + y*y + z*z);        // 3D range

            telemetry.addData("Tag ID", f.getFiducialId());
            telemetry.addData("Forward (m)", "%.3f", forwardMeters);
            telemetry.addData("Range (m)",   "%.3f", rangeMeters);
        } else {
            telemetry.addData("Tag", "none");
        }
        telemetry.update();
    }


}
