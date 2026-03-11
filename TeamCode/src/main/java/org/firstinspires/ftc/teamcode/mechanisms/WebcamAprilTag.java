/*
   From Coach Pratt "FTC AprilTags: Get Distance & Angle (Standard Webcam)" YouTube video on improving programming techniques.
   https://youtu.be/OZt33z-lyYo?si=3r6IRNzg7J8hRnAA
*/
package org.firstinspires.ftc.teamcode.mechanisms;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class WebcamAprilTag {

    private VisionPortal visionPortal;  // used to manage the video source.
    private AprilTagProcessor aprilTagProcessor;    // processor in FTC SDK used for managing the AprilTag detection process.

    private List<AprilTagDetection> currentDetections = new ArrayList<>();

    private Telemetry telemetry;

    /*
     Initialize hardware and telemetry.
    */
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();  // create an AprilTagProcessor using default settings.
        aprilTagProcessor = new AprilTagProcessor.Builder() // create and initialize an AprilTag processor

                // The following default settings are available to un-comment and edit as needed.
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)

                // ----- Camera Calibration -----
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTagProcessor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }

    // Read AprilTag(s).
    public void update() {
        currentDetections = aprilTagProcessor.getDetections();
    }

     // Return list of AprilTags detected by the AprilTag processor.
    public List<AprilTagDetection> getCurrentDetections() {
        return currentDetections;
    }

     // Return results from specified AprilTag.  Return NULL if specified tag is not found.
    public AprilTagDetection getAprilTagByID(int id) {
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == id) {
                return detection;   // return detected tag
            }
        }
        return null;    // return null if no tag found
    }

     // Send detected AprilTag information to Driver Hub display.
    public void telemetryAprilTag(AprilTagDetection detection) {
        if (detection == null) {
            return;
        }

        // Display info on detected AprilTag
        if (detection.metadata != null) {
/*
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("X Y Z %6.1f %6.1f %6.1f (cm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("P R Y %6.1f %6.1f %6.1f (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetry.addLine(String.format("R B E %6.1f %6.1f %6.1f (cm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
*/
            telemetry.addData("\nAprilTag ID", "%d %s", detection.id, detection.metadata.name);
            telemetry.addData("X Y Z", "%6.1f %6.1f %6.1f  (cm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
            telemetry.addData("P R Y", "%6.1f %6.1f %6.1f  (degrees)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);
            telemetry.addData("R B E", "%6.1f %6.1f %6.1f  (cm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);

        } else {
            telemetry.addData("\nAprilTag ID", "%d Unknown", detection.id);
            telemetry.addData("Center", "%6.0f %6.0f  (pixels)", detection.center.x, detection.center.y);
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nKey:\nX Y Z = +X (right), +Y (forward), +Z (up) dist.");
        telemetry.addLine("P R Y = Pitch, Roll & Yaw (XYZ rotation)");
        telemetry.addLine("R B E = Range, Bearing & Elevation");
    }

     // Pause vision portal streaming to conserve resources.
    public void pauseVisionPortal() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

     // Resume vision portal streaming.
    public void resumeVisionPortal() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

     // Stop active vision portal when no longer needed.
    public void stopVisionPortal() {
        if (visionPortal != null) { // vision portal instance is active
            visionPortal.close();
        }
    }
}
