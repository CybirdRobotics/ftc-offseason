/*
   From Coach Pratt "FTC AprilTags: Get Distance & Angle (Standard Webcam)" YouTube video on improving programming techniques.
   https://youtu.be/OZt33z-lyYo?si=3r6IRNzg7J8hRnAA
*/
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.WebcamAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="April Tag Example (Webcam)", group = "OpMode")
@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class AprilTagExample extends LinearOpMode {

    private final WebcamAprilTag webcamAprilTag = new WebcamAprilTag();
    private static final int TARGET_APRILTAG_ID = 20;   // change to the ID of your desired AprilTag

    @Override
    public void runOpMode() throws InterruptedException {

        webcamAprilTag.init(hardwareMap, telemetry);

        // Wait for the Driver Station start button to be pressed.
        telemetry.addData("Camera preview on/off", "DS | 3 dots | Camera Stream");
        telemetry.addData(">", "Press ▶ to start OpMode.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Get AprilTag information
            webcamAprilTag.update();    // update the vision portal to read AprilTag(s)

            AprilTagDetection aprilTagID = webcamAprilTag.getAprilTagByID(TARGET_APRILTAG_ID);
            webcamAprilTag.telemetryAprilTag(aprilTagID);

            // Push telemetry to the Driver Station display
            telemetry.update();

            // Use the gamepad directional pad to turn streaming on/off as needed
            if (gamepad1.dpad_down) {
                webcamAprilTag.pauseVisionPortal();
            } else if (gamepad1.dpad_up) {
                webcamAprilTag.resumeVisionPortal();
            }

            // Share the CPU
            sleep(20);
        }

        // Stop the vision portal to save CPU resources when camera is no longer needed
        webcamAprilTag.stopVisionPortal();

    }   // end method runOpMode()
}