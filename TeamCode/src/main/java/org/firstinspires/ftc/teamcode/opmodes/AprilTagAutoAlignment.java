package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AprilTagAutoAlignment extends OpMode {

    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // Target AprilTag ID
    private static final int TARGET_APRILTAG_ID = 20;   // change to the ID of your desired AprilTag
    private final MecanumDrive drive = new MecanumDrive();

    // --------- PD Controller ---------
    double Kp = 0.002;  // proportional constant
    double Kd = 0.0001; // derivative constant
    double error = 0;
    double lastError = 0;
    double targetX = 0;     // offset
    double angleTolerance = 0.4;
    double currentTime = 0;
    double lastTime = 0;

    // --------- Drive Setup ---------
    double forward, strafe, turn;

    // --------- Controller Based PID Tuning ---------
    // Allows up to update PD parameters using a gamepad.  Not needed if using Panels.
    double[] stepSizes = {0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        drive.init(hardwareMap);

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press PLAY to start.");
    }

    public void start() {
        resetRuntime();
        currentTime = getRuntime();
    }

    @Override
    public void loop() {

        // --------- Get Mecanum Drive Inputs ---------
        // FORWARD/BACKWARD or LONGITUDINAL = left joystick y axis (robot centric)
        double forward = -gamepad1.left_stick_y;  // set as negative so pushing joystick forward is a positive value
        // STRAFE = left joystick x axis
        double strafe = gamepad1.left_stick_x * 1.1;    // multiplier to counteract imperfect strafing, adjustable based on driver preference
        // TURN = right joystick x axis
        double turn = gamepad1.right_stick_x;

        // --------- Get AprilTag Info ---------
        // Update the vision portal
        aprilTagWebcam.update();

        AprilTagDetection targetAprilTagID = aprilTagWebcam.getTagBySpecificID(TARGET_APRILTAG_ID);  // target AprilTag 20
        telemetry.addData("AprilTag ID" + TARGET_APRILTAG_ID, targetAprilTagID.toString());

        // --------- Auto Align Logic ---------
        if (gamepad1.left_trigger > 0.3) {
            if (targetAprilTagID != null) {
                error = targetX - targetAprilTagID.ftcPose.bearing; // equivalent to Limelight Tx

                if (Math.abs(error) < angleTolerance) {
                    turn = 0;
                } else {
                    double pTerm = error * Kp;

                    currentTime = getRuntime();
                    double dT = currentTime - lastTime;
                    double dTerm = ((error = lastError) / dT) * Kd);

                    turn = Range.clip(pTerm * dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = currentTime;
                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            lastTime = getRuntime();
            lastError = 0;
        }

        // --------- Move Robot ---------
        drive.driveFieldRelative(forward, strafe, turn);

        // --------- Update Kp and Kd On The Fly ---------
        // Button 'B' or 'Circle' cycles through the different step sizes for PD tuning
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex +1) % stepSizes.length;  // modulo wraps the index back to 0
        }

        // 'D-Pad' up/down adjusts the Kp gain
        if (gamepad1.dpadUpWasPressed()) {
            Kp += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            Kp -= stepSizes[stepIndex];
        }

        // 'D-Pad' left/right adjusts the Kd gain
        if (gamepad1.dpadLeftWasPressed()) {
            Kd -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            Kd += stepSizes[stepIndex];
        }

        // --------- Telemetry ---------
        if (targetAprilTagID != null) {
            if (gamepad1.left_trigger > 0.3) {
                telemetry.addLine("AUTO Align Mode");
            }
            aprilTagWebcam.telemetryAprilTag(targetAprilTagID);
            telemetry.addData("Error", error);
        } else {
            telemetry.addLine("MANUAL Mode");
        }
        telemetry.addLine("-----------------------");
        telemetry.addData("Tuning Kp", "%.4f (D-Pad Up/Down)", Kp);
        telemetry.addData("Tuning Kd", "%.4f (D-Pad Left/Right)", Kp);
        telemetry.addData("Step Size", "%.4f (B/Circle Button)", stepSizes[stepIndex]);

    }
}
