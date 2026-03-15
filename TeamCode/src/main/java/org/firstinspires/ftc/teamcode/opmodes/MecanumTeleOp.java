/*
    From FTC Team 7477 - FTC Programming Episode 8: Mecanum Drive (robot centric)
                         FTC Programming Episode 9: Scaling Drive Powers Proportionally

    and https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#robot-centric-final-sample-code
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.LimelightAprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name="Mecanum TeleOp (w Limelight)", group = "OpMode")
//@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class MecanumTeleOp extends LinearOpMode {

    // Create an instance of the Limelight object to be used to access camera functionality.
    LimelightAprilTag limelight = new LimelightAprilTag(); // create instance of LimeLight object

    // Create an instance of the MecanumDrive object to be used to access robot drive functionality.
    MecanumDrive drive = new MecanumDrive();    // create instance of MecanumDrive object

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);

        try {
            limelight.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Limelight camera.");
            telemetry.update();
            return;
        }

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press PLAY to start.");
        telemetry.update();

        waitForStart();

        limelight.start();  // start polling Limelight for data.

        // Run until the driver presses stop
        while (opModeIsActive()) {
            double KpTurn = 0.05;     // proportional control constant for turn
            double KpDistance = 0.1;  // proportional control constant for distance
            double distanceError = 0, headingError = 0;
            double distance = 0;
            double targetDistance = 51; // desired distance to target in inches

            telemetry.addLine("Defaults to FIELD relative mode.");
            telemetry.addLine("Hold LEFT bumper to drive in ROBOT relative mode.");
            telemetry.addLine("Hold RIGHT bumper for AUTO SCORING alignment.");
            telemetry.addLine("Press A to reset YAW.");

            // When gamepad1.a is pressed, reset the Yaw to 0 based on the orientation relative to the robot's position
            if (gamepad1.a) {
                drive.resetYaw();
            }
/*
            // Limelight MegaTag2 requires input (yaw) from the IMU for localization.
            YawPitchRollAngles orientation = drive.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
*/
            // Get the current Limelight pipeline results
            LLResult result = limelight.getLLResults();
            if (result != null && result.isValid()) {
                // The coordinate system for botPose matches the standard FTC coordinate system. (0,0,0) is the center of the field floor
                // For non-diamond configurations, 0 degrees Yaw means the blue alliance is on the left side of your robot, and the red alliance is on the right side of your robot.
                double tx = result.getTx(); // how far left or right the target is (degrees)
                double ty = result.getTy(); // how far up or down the target is (degrees)
                double ta = result.getTa(); // how big the target looks relative to the frame of view
                distance = limelight.getDistance(ty); // calculate the estimated distance to target
                distanceError = (distance - targetDistance) * KpDistance;
                headingError = (tx * KpTurn);   // implement PID control logic to center on the AprilTag. For example, adjust 'turn' based on 'tx'

                // Send telemetry data to the Driver Hub
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.addData("Distance to Target", distance);
/*
                Pose3D botpose = result.getBotpose_MT2();   // get the 3D object position and orientation
                if (botpose != null) {
                    telemetry.addData("\nBotpose", botpose.toString());
                } else {
                    telemetry.addLine("No Botpose data available.");
                }
*/
            } else {
                telemetry.addLine("No valid target detected.");
            }

            if (gamepad1.right_bumper) {    // align robot with target, then press right_bumper for auto shooting alignment
                drive.driveRobotRelative(distanceError, 0, headingError);
            } else if (gamepad1.left_bumper) {  // press the left bumper for robot centric drive
                drive.driveRobotRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
            } else {    // field centric drive
                drive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
            }
            telemetry.update();
        }

        if (isStopRequested()) {
            limelight.stop();   // stop Limelight data polling.
        }
    }
}