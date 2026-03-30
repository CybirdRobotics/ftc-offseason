package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous
public class AutoSquareByEncoder extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();    // create instance of MecanumDrive object to access robot drive functionality
    private ElapsedTime runtime = new ElapsedTime();

    private static final double TIMEOUT_SECONDS = 8.0;  // movement timeout in seconds

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready. Press PLAY to start.");
        telemetry.update();

        waitForStart();

        // Step 1: drive forward 24 inches
        //encoderDrive(DRIVE_SPEED, 24, 24, 5.0);  // Drive forward 24 inches
        drive.driveEncoder(0.8, 24, DistanceUnit.INCH); // drive forward at 80% speed
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < TIMEOUT_SECONDS) {
            telemetry.addData("Path", "Step 1: current position %d ticks.", drive.getCurrentPosition());
            telemetry.update();
        }

        // Step 2: turn right 90 degrees
        drive.driveRobotRelative(0,0,0.4);  // turn right at 40% speed
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.2)) {
            telemetry.addData("Path", "Step 2: %4.1f s elapsed.", runtime.seconds());
            telemetry.update();
        }

        // Step n: stop movement
        drive.stopRobot();

        telemetry.addData("Path", "Complete.");
        telemetry.update();
        sleep(1000);
    }
}
