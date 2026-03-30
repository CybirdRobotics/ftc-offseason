package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous
public class AutoSquareByTime extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();    // create instance of MecanumDrive object to access robot drive functionality
    private ElapsedTime driveTimer = new ElapsedTime(); // create a timer


    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press PLAY to start.");
        telemetry.update();

        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // Step 1: drive forward for 24 inches
        drive.driveRobotRelative(0.8,0,0);  // drive forward at 80% speed
        driveTimer.reset();
        while (opModeIsActive() && (driveTimer.seconds() < 2.0)) {
            telemetry.addData("Path", "Step 1: %3.1fs elapsed.", driveTimer.seconds());
            telemetry.update();
        }

        // Step 2: turn right 90 degrees
        drive.driveRobotRelative(0,0,0.4);  // turn right at 40% speed
        driveTimer.reset();
        while (opModeIsActive() && (driveTimer.seconds() < 1.2)) {
            telemetry.addData("Path", "Step 2: %3.1fs elapsed.", driveTimer.seconds());
            telemetry.update();
        }

        // Step n: stop movement
        drive.stopRobot();

        telemetry.addData("Path", "Complete.");
        telemetry.update();
        sleep(1000);    // wait for 1 second
    }
}