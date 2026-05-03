package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous
public class AutoSquareByEncoder extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();    // create MecanumDrive object to access robot drive functionality

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);    // hardwaremap

        // Set the initial starting point (location) of the robot
        drive.setPinpointPosition(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press PLAY to start.");
        telemetry.update();

        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way
        double[] targets = {-90, -180, 90, 0};  // normalized target headings for each corner of the square, moving in a clock-wise direction.

        int k = 0;  // initialize the step counter
        for (double targetHeading : targets) {
            // Drive forward for 48 inches
            k++;    // increment step counter
            drive.encoderDrive(0.6, 36, DistanceUnit.INCH); // drive forward 36 inches at 70% speed
            while (opModeIsActive() && drive.isBusy()) {
                telemetry.addData("Path", "Step %d: current position %d ticks.", k, drive.getCurrentPosition());
                telemetry.update();
            }
/*
            drive.encoderDrive(0.6, -36, DistanceUnit.INCH); // drive backward (use negative distance, not speed).
            while (opModeIsActive() && drive.isBusy()) {
                telemetry.addData("Path", "Step %d: current position %d ticks.", k, drive.getCurrentPosition());
                telemetry.update();
            }
*/
            // Turn right 90 degrees
            k++;    // increment step counter
            drive.driveRobotRelative(0, 0, 0.5);  // start turning right at 50% speed
            while (opModeIsActive() && (Math.abs(targetHeading - drive.getHeading()) > 3.0)) { // loop until within error tolerance (i.e., 2 degrees)
                telemetry.addData("Path", "Step %d", k);
                telemetry.addData("Target Heading (Normalized)", targetHeading);
                telemetry.addData("Current Heading", drive.getHeading());
                telemetry.addData("Error", (targetHeading - drive.getHeading()));
                telemetry.update();
            }
        }

        // Step n: stop movement
        drive.stopRobot();

        telemetry.addData("Path", "Complete.");
        telemetry.update();
        sleep(1000);    // wait for 1 second
    }
}