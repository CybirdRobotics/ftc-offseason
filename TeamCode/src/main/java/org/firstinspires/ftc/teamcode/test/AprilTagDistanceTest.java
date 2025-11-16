package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="AprilTag Distance Test", group="Test")
@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class AprilTagDistanceTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private double range;

    @Override
    public void init() {

        imu = hardwareMap.get(IMU.class, "imu");
        // TODO: Change the the following to match the controller Hub orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Limelight camera.");
            telemetry.update();
            return;
        }

        limelight.pipelineSwitch(9);    // specifies which pipeline to use, in this case pipeline 9 is configured for AprilTag 20.
        limelight.setPollRateHz(100);   // set how often we ask Limelight for data (100 times per second)

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press play to start.");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        limelight.start();  // start polling Limelight for data.
    }

    @Override
    public void loop() {
        // for Limelight MegaTag2, tell Limelight which way your robot is facing
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            range = getDistance(result.getTy());

            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta", result.getTa());
            telemetry.addData("Botpose", botpose.toString());
            telemetry.addLine();
            telemetry.addData("Range to AprilTag", range);
            telemetry.addData("Bearing to AprilTag", botpose.getOrientation().getYaw());
            telemetry.update();
        }
    }

    public double getDistance(double ty) {
        // Calculate distance to target AprilTag.
        // From the Limelight documentation, https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance.
        // Solve for distance = (h2-h1) / tan(a1+a2) where the height of the target (h2) is known because it is a property of the field (e.g. AprilTag),
        // the height of your camera lens above the floor (h1) is known as is its mounting angle (a1).
        // The Limelight can tell you the Ty angle to the target (a2).  Note, The tan() usually expects an input measured in radians.
        // To convert an angle measurement from degrees to radians, multiply by (3.14159/180.0).

        // degrees your limelight is pitched back (rotated) from vertical
        double limelightPitchAngle = 14.0;

        // distance from the center of the Limelight lens to the floor in inches
        double limelightHeight = 4.0;

        // distance from the target to the floor in inches
        double targetHeight = 38.75 - 9.25; // center of AprilTag on DECODE goal. Note, goal is 38.75 in tall and AprilTag center is located 9.25 in from top of goal (CM pg. 77).

        double angleToTargetDegrees = limelightPitchAngle + ty;
        double angleToTargetRadians = angleToTargetDegrees * (3.14159 / 180.0);

        // calculate distance in inches
        return (targetHeight - limelightHeight) / Math.tan(angleToTargetRadians);
    }

    @Override
    public void stop() {
        super.stop();
        limelight.stop();
    }
}
