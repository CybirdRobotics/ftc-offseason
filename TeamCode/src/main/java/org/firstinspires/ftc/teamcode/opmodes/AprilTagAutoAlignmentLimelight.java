package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightAprilTag;

@TeleOp(name="April Tag Alignment (Limelight)", group = "OpMode")
@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class AprilTagAutoAlignmentLimelight extends LinearOpMode {

    // ... define limelight variables ...
    private final LimelightAprilTag limelightAprilTag = new LimelightAprilTag();
    // Target AprilTag ID
    private static final int TARGET_APRILTAG_ID = 20;   // change to the ID of your desired AprilTag

    @Override
    public void runOpMode() throws InterruptedException {

       limelightAprilTag.init(hardwareMap, telemetry);


        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press play to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            limelightAprilTag.getLLStatus();

            LLResult result = limelightAprilTag.getLLResults();

            Pose3D botpose_mt1 = result.getBotpose();   // MegaTag 1
            if (botpose_mt1 != null) {
                double x = botpose_mt1.getPosition().x;
                double y = botpose_mt1.getPosition().y;
                telemetry.addData("BotPose_MT1", botpose_mt1.toString());
                telemetry.addData("Yaw (BotPos)", botpose_mt1.getOrientation().getYaw());
                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            }

            telemetry.update();
        }
    }
}
