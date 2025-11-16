/*
    From FTC Team 7477 - FTC Programming Episode 8: Mechanum Drive (robot centric)
                         FTC Programming Episode 9: Scaling Drive Powers Proportionally

    and https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#robot-centric-final-sample-code
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Sensors;

@TeleOp(name="Mecanum Drive (Field Relative)", group = "OpMode")
//@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class MecanumTeleOpFieldRelative extends LinearOpMode {

    private Limelight3A limelight;

    MecanumDrive drive = new MecanumDrive();    // create instance of MecanumDrive object (class)
    Sensors sensors = new Sensors();    // create instance of Sensors object for IMU

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        sensors.init(hardwareMap);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Limelight camera.");
            telemetry.update();
            return;
        }

        limelight.pipelineSwitch(9);    // specifies which pipeline to use, in this case pipeline 9 is configured for AprilTag 20.
        //limelight.setPollRateHz(100);   // set how often we ask Limelight for data (100 times per second)

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
            double targetDistance = 24; // desired distance to target in inches

            telemetry.addLine("Press A to reset Yaw.");
            telemetry.addLine("Hold LEFT bumper to drive in robot relative mode.");
            telemetry.addLine("Hold RIGHT bumper for auto scoring alignment.");

            // When gamepad1.a is pressed, reset the Yaw to 0 based on the orientation relative to the robot's position
            if (gamepad1.a) {
                sensors.resetYaw();
            }

            // Limelight MegaTag2 requires input (yaw) from the IMU for localization.
            YawPitchRollAngles orientation = sensors.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            // Get the current Limelight pipeline results
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // The coordinate system for botPose matches the standard FTC coordinate system. (0,0,0) is the center of the field floor
                // For non-diamond configurations, 0 degrees Yaw means the blue alliance is on the left side of your robot, and the red alliance is on the right side of your robot.
                double tx = result.getTx(); // how far left or right the target is (degrees)
                double ty = result.getTy(); // how far up or down the target is (degrees)
                double ta = result.getTa(); // how big the target looks relative to the frame of view
                distance = getDistance(ty); // calculate the estimated distance to target
                distanceError = (distance - targetDistance) * KpDistance;
                headingError = (tx * KpTurn);   // implement PID control logic to center on the AprilTag. For example, adjust 'turn' based on 'tx'

                // Send telemetry data to the Driver Hub
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.addData("Distance to Target", distance);

                Pose3D botpose = result.getBotpose_MT2();   // get the 3D object position and orientation
                if (botpose != null) {
                    telemetry.addData("\nBotpose", botpose.toString());
                } else {
                    telemetry.addLine("No Botpose data available.");
                }
            } else {
                telemetry.addLine("No AprilTag detected.");
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

    public double getDistance(double a2) {
        // Calculate distance to target AprilTag.
        // From the Limelight documentation, https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance.
        // Solve for distance = (h2-h1) / tan(a1+a2) where the height of the target (h2) is known because it is a property of the field (e.g. AprilTag),
        // the height of your camera lens above the floor (h1) is known as is its mounting angle (a1).
        // The Limelight can tell you the Ty angle to the target (a2).  Note, The tan() usually expects an input measured in radians.
        // To convert an angle measurement from degrees to radians, multiply by (3.14159/180.0).

        // Degrees your limelight is pitched back (rotated) from vertical
        double limelightPitchAngle = 15;    // 15 degrees in LL Pitch config

        // Distance from the center of the Limelight lens to the floor in inches
        double limelightHeight = 2.5;   // 0.0635 meters in LL Up config

        // Distance from the center of the robot to the Limelight lens in inches
        double limelightForwardOffset = 6.375;  // 0.161925 meters in LL Forward config

        // Distance from the target to the floor in inches
        // Note, goal is 38.75 in (98.45 cm) tall and AprilTag center is located 9.25 in (23.5 cm) from top of goal (CM pgs. 69 & 77).
        double targetHeight = 29.5; // center of AprilTag on DECODE goal in inches (38.75 - 9.25)

        double angleToTargetDegrees = limelightPitchAngle + a2;
        double angleToTargetRadians = angleToTargetDegrees * (3.14159 / 180.0);

        // calculate distance in inches
        return (targetHeight - limelightHeight) / Math.tan(angleToTargetRadians);
    }
}