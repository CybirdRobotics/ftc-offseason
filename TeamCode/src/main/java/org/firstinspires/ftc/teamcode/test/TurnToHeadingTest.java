package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "Mecanum Turn To Heading Test", group = "Test")
public class TurnToHeadingTest extends LinearOpMode {

    private final MecanumDrive drive = new MecanumDrive(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and zero the IMU yaw
        drive.init(hardwareMap);

        telemetry.addLine("Robot initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // 1. Turn 90 degrees counter-clockwise at max 60% power
            drive.turnToHeading(90.0, 0.6);
            sleep(500);

            // 2. Turn to -90 degrees (180-degree turn clockwise)
            drive.turnToHeading(-90.0, 0.6);
            sleep(500);

            // 3. Return to starting orientation (0 degrees)
            drive.turnToHeading(0.0, 0.6);
        }
    }
}
