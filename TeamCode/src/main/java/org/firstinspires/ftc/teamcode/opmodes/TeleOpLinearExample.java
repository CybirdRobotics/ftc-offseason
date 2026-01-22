/*
   From Coach Pratt "GoBILDAs DECODE Starter Robot is Good.  Let's Make it Great" YouTube video on improving programming techniques.
   https://youtu.be/zfPU8Vy9yiY?si=A0EJxVQ1khANI9vK
*/

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name="TeleOp (LinearOpMode) Example", group = "OpMode")
//@Disabled // comment this out to add to the OpMode list on the Driver Hub
public class TeleOpLinearExample extends LinearOpMode {

    // Create a MecanumDrive object to be used to access robot drive functionality.
    // Note, prefix any mecanum drive functions with "drive." to access this class.
    MecanumDrive drive = new MecanumDrive();    // create instance of MecanumDrive object (class)

    @Override
    public void runOpMode() {

        // Initialize the drive motors and IMU, using the mecanum drive class
        drive.init(hardwareMap);

        // Send telemetry message to signify robot is ready and wait for driver to press START.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press PLAY to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Press A to reset Yaw.");
            telemetry.addLine("Hold LEFT bumper to drive in robot relative mode.");

            // When gamepad1.a is pressed, reset the YAW to 0 based on the orientation relative to the robot's position
            if (gamepad1.a) {
                drive.resetYaw();
            }

            if (gamepad1.left_bumper) {  // press the left bumper for robot centric drive
                drive.driveRobotRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            } else {    // field centric drive
                drive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            telemetry.addData("\nSpeed Limiter", drive.speedLimiter);
            telemetry.update();
        }
    }
}
