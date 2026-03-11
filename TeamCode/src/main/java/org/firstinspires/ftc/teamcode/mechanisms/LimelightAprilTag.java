package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightAprilTag {

    // Limelight object (replace with your actual Limelight integration)
    // For example, you might have a custom Limelight class or use a library
    private Limelight3A limelight;

    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize Limelight (replace with your actual initialization)
        telemetry.addLine("Initializing Limelight 3A...\n");
        try {
            limelight = hwMap.get(Limelight3A.class, "Limelight 3A");
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Limelight.");
            telemetry.update();
            return;
        }

        limelight.pipelineSwitch(9);    // specifies which pipeline to use
        limelight.setPollRateHz(100);   // from LL documentation to  set how often we ask Limelight for data (100 times per second)
        limelight.start();  // start polling Limelight for data.
    }

    public void start() {
        limelight.start();  // start polling Limelight for data.
    }

    public void getLLStatus() {
        LLStatus status = limelight.getStatus();
        telemetry.addLine("Limelight Status");
        telemetry.addData("Device Name", "%s", status.getName());
        telemetry.addData("Pipeline", "Index: %d, Type: %s\n",
                status.getPipelineIndex(), status.getPipelineType());
    }

    public LLResult getLLResults() {
        // Get Limelight results (replace with your actual Limelight API calls)
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // The coordinate system for botPose matches the standard FTC coordinate system. (0,0,0) is the center of the field floor
            // For non-diamond configurations, 0 degrees Yaw means the blue alliance is on the left side of your robot, and the red alliance is on the right side of your robot.
            double tx = result.getTx(); // how far left or right the target is (degrees)
            double ty = result.getTy(); // how far up or down the target is (degrees)
            double ta = result.getTa(); // how big the target looks (0%-100% of the image)
            telemetry.addData("Tx", tx);
            telemetry.addData("Ty", ty);
            telemetry.addData("Ta", "%0.4f", ta + "\n");
        }

        return result;
    }

    public double getDistance(double a2) {
        // Calculate distance to target AprilTag.
        // From the Limelight documentation, https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance.
        // Solve for distance = (h2-h1) / tan(a1+a2) where the height of the target (h2) is known because it is a property of the field (e.g. AprilTag),
        // the height of your camera lens above the floor (h1) is known as is its mounting angle (a1).
        // The Limelight can tell you the Ty angle to the target (a2).  Note, The tan() usually expects an input measured in radians.
        // To convert an angle measurement from degrees to radians, multiply by (pi/180.0).

        // Degrees your limelight is pitched back (rotated) from vertical
        double CAMERA_PITCH_ANGLE = 15;    // 15 degrees in LL Pitch config

        // Distance from the center of the Limelight lens to the floor in inches
        double CAMERA_HEIGHT = 2.5;   // 0.0635 meters in LL Up config

        // Distance from the center of the robot to the Limelight lens in inches
        double CAMERA_FORWARD_OFFSET = 6.375;  // 0.161925 meters in LL Forward config

        // Distance from the target to the floor in inches
        // Note, DECODE goal is 38.75 in (98.45 cm) tall and AprilTag center is located 9.25 in (23.5 cm) from top of goal (CM pgs. 69 & 77).
        double TARGET_HEIGHT = 29.5; // center of AprilTag on DECODE goal in inches (38.75 - 9.25)

        double angleToTargetDegrees = CAMERA_PITCH_ANGLE + a2;

        // calculate distance in inches
        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(Math.toRadians(angleToTargetDegrees));
    }

    public void stop() {
        limelight.stop();   // stop Limelight data polling
    }
}