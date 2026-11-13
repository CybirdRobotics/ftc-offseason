package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous
public class AutoSquareByEncoder extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive(telemetry);    // create MecanumDrive object to access robot drive functionality

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);    // hardware map

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
            // Drive forward 48 inches
            k++;    // increment step counter

            drive.driveByEncoder(24, DistanceUnit.INCH, 0.7); // drive forward 70% speed
            while (opModeIsActive() && drive.isMotorBusy()) {
                telemetry.addData("Path", "Step %d: current position %d ticks.", k, drive.getCurrentPosition());
                telemetry.update();
            }

            drive.driveByEncoder(-24, DistanceUnit.INCH, 0.6); // to drive backward, use negative distance, not speed.
            while (opModeIsActive() && drive.isMotorBusy()) {
                telemetry.addData("Path", "Step %d: current position %d ticks.", k, drive.getCurrentPosition());
                telemetry.update();
            }

            // Turn right 90 degrees
            //drive.driveRobotRelative(0, 0, 0.5);  // start turning right at 50% speed
            //while (opModeIsActive() && (Math.abs(targetHeading - drive.getHeading(AngleUnit.DEGREES)) > 2.0)) { // loop until within error tolerance (i.e., 2 degrees)
            drive.turnToHeading(targets[k-1], 0.4);
            while (opModeIsActive() && drive.isMotorBusy()) {
                telemetry.addData("Path", "Step %d", k);
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