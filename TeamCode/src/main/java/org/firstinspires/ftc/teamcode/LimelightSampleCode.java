/*
    From the Limelight 3A documentation.
    Ref: https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a#7-ftc-programming
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Limelight Sample Code", group = "Limelight")
@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class LimelightSampleCode extends LinearOpMode {

    private Limelight3A limelight;  // Instantiate the Limelight 3A object.
    private IMU imu;    // Declare the IMU object.

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the IMU with the correct Hub orientation for your robot.
        imu = hardwareMap.get(IMU.class, "imu");
        // TODO: Change the the following to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Limelight.");
            telemetry.update();
            return;
        }

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(9);    // specifies which pipeline to use
        limelight.setPollRateHz(100);   // set how often we ask Limelight for data (100 times per second)
        limelight.start();  // start polling Limelight for data.

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press play to start.");
        telemetry.update();

        waitForStart();

        limelight.start();  // start polling Limelight for data.

        while (opModeIsActive()) {
            // for Limelight MegaTag2, tell Limelight which way your robot is facing
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());

            LLStatus status = limelight.getStatus();
            telemetry.addData("Device Name", "%s", status.getName());
            telemetry.addData("Pipeline", "Index: %d, Type: %s\n",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addLine();

                // The coordinate system for botPose matches the standard FTC coordinate system. (0,0,0) is the center of the field floor
                // For non-diamond configurations, 0 degrees Yaw means the blue alliance is on the left side of your robot, and the red alliance is on the right side of your robot.
                double tx = result.getTx(); // how far left or right the target is (degrees)
                double ty = result.getTy(); // how far up or down the target is (degrees)
                double ta = result.getTa(); // how big the target looks (0%-100% of the image)
                telemetry.addData("Tx", tx);
                telemetry.addData("Ty", ty);
                telemetry.addData("Ta", ta);

                Pose3D botpose_mt1 = result.getBotpose();   // MegaTag 1
                if (botpose_mt1 != null) {
                    double x = botpose_mt1.getPosition().x;
                    double y = botpose_mt1.getPosition().y;
                    telemetry.addData("BotPose_MT1", botpose_mt1.toString());
                    telemetry.addData("Yaw (BotPos)", botpose_mt1.getOrientation().getYaw());
                    telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                }

                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    telemetry.addData("BotPose_MT2", botpose_mt2.toString());
                    telemetry.addData("Yaw (BotPos)", botpose_mt2.getOrientation().getYaw());
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                }
            } else {
                telemetry.addData("Limelight", "No targets found.");
            }
            telemetry.update();
        }
        limelight.stop();   // Stop Limelight data polling.
    }

/*
    private void processAprilTags() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int tagId = fr.getFiducialId();
                    Pose3D botPosField = fr.getRobotPoseFieldSpace();
                    if (tagId != 0) {
                        telemetry.addData("AprilTag ID", tagId);
                        telemetry.addData("BotPose", botPose.toString());
                        telemetry.addData("BotPose (Field)", botPosField.getPosition());
                        telemetry.addData("Capture Latency", "%.2f ms", captureLatency);
                        telemetry.addData("Targeting Latency", "%.2f ms", targetingLatency);
                    } else {
                        telemetry.addData("AprilTag ID", tagId);
                        telemetry.addData("Pose3D", "No pose data available.");
                    }
                }
            } else {
                telemetry.addData("AprilTag", "No AprilTag detected.");
            }
        } else {
            telemetry.addData("Limelight", "No valid result.");
        }
    }
 */

}