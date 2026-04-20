package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Gyroscope;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous
public class AutoSquareByGyro extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();    // create MecanumDrive object to access robot drive functionality
    Gyroscope gyro = new Gyroscope();   // create Gyroscope (IMU) object
    private ElapsedTime driveTimer = new ElapsedTime(); // create timer object


    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        gyro.init(hardwareMap);

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press PLAY to start.");
        telemetry.update();

        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way
        double[] targets = {-90, 180, 90, 0};  // normalized target headings for each corner of the square, moving in a clock-wise direction.

        int k = 0;  // initialize the step counter
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
            while (opModeIsActive() && (Math.abs(targetHeading - gyro.getYaw()) > 4.0)) { // loop until within error tolerance (i.e., 2 degrees)
                telemetry.addData("Path", "Step %d", k);
                telemetry.addData("Target Heading (Normalized)", targetHeading);
                telemetry.addData("Current Heading", gyro.getYaw());
                telemetry.addData("Error", (targetHeading - gyro.getYaw()));
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