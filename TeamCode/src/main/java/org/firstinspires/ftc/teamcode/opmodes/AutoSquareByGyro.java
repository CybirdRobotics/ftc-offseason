package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous
public class AutoSquareByGyro extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();    // create instance of MecanumDrive object to access robot drive functionality
    private ElapsedTime driveTimer = new ElapsedTime(); // create a timer


    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);

        drive.resetYaw();

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press PLAY to start.");
        telemetry.update();

        waitForStart();

        /* To turn 90 degrees without resetting YAW with each turn, use the formula target = start yaw + 90, then normalize
           the result to stay within the [-180, 180] degree range of the REV IMU.  For example:
              • corner 1: target = 90 degrees
              • corner 2: target = 180 degrees
              • corner 3: target = 270 degrees (normalized to -90)
              • corner 4: target = 360 degrees (normalized to 0)
         */
        double[] targets = {-90, 180, 90, 0};  // normalized target headings for each corner of the square, moving in a clock-wise direction.

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way
        int k = 0;
        for (double targetHeading : targets) {
            // Step 1: drive forward for 48 inches
            k++;
            drive.driveRobotRelative(0.7, 0, 0);  // start driving forward at 80% speed
            driveTimer.reset();
            while (opModeIsActive() && (driveTimer.seconds() < 1.8)) {
                telemetry.addData("Path", "Step %d: %3.1fs elapsed.", k, driveTimer.seconds());
                telemetry.update();
            }

            // Step 2: turn right 90 degrees
            k++;
            drive.driveRobotRelative(0, 0, 0.6);  // start turning right at 50% speed
            while (opModeIsActive() && (Math.abs(targetHeading - drive.getHeading()) > 2.0)) { // loop until within error tolerance (i.e., 2 degrees)
                telemetry.addData("Path", "Step %d", k);
                telemetry.addData("Target Heading (Normalized)", targetHeading);
                telemetry.addData("Current Heading", drive.getHeading());
                telemetry.addData("Error", (targetHeading - drive.getHeading()));
                telemetry.update();
            }
            drive.stopRobot();
        }

        // Step n: stop movement
        drive.stopRobot();

        telemetry.addData("Path", "Complete.");
        telemetry.update();
        sleep(1000);    // wait for 1 second
    }
}