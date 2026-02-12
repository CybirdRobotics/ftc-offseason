/*
   From Coach Pratt "FTC AprilTags: Get Distance & Angle (Standard Webcam)" YouTube video on improving programming techniques.
   https://youtu.be/OZt33z-lyYo?si=3r6IRNzg7J8hRnAA
*/
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="AprilTag Webcam Test", group = "Test")
//@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class AprilTagWebcamTest extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // Target AprilTag ID
    private static final int TARGET_APRILTAG_ID = 20;   // change to the ID of your desired AprilTag

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // Update the vision portal
        aprilTagWebcam.update();

        AprilTagDetection targetAprilTagID = aprilTagWebcam.getTagBySpecificID(TARGET_APRILTAG_ID);  // target AprilTag 20
        telemetry.addData("AprilTag ID" + TARGET_APRILTAG_ID, targetAprilTagID.toString());
    }
}
