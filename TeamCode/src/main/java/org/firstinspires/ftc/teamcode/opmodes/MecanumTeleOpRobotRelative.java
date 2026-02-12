/*
    From FTC Team 7477 - FTC Programming Episode 8: Mechanum Drive (robot centric)
                         FTC Programming Episode 9: Scaling Drive Powers Proportionally

    and https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#robot-centric-final-sample-code
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Mecanum Drive (Robot Relative)", group = "OpMode")
@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class MecanumTeleOpRobotRelative extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings specified here as parameters
        // must match the names assigned in the robot controller configuration.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // TODO: Make sure all motors are facing the correct direction.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Use encoder for constant power resulting in increased accuracy
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot ready.  Press play to start.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Run until the driver presses stop
        while (opModeIsActive()) {

            // FORWARD/BACKWARD or LONGITUDINAL = left joystick y axis (robot centric)
            double forward = -gamepad1.left_stick_y;  // set as negative so pushing joystick forward is a positive value
            // STRAFE = left joystick x axis
            double strafe = gamepad1.left_stick_x * 1.1;    // multiplier to counteract imperfect strafing, adjustable based on driver preference
            // TURN = right joystick x axis
            double turn = gamepad1.right_stick_x;

            driveRobotRelative(forward, strafe, turn);
        }
    }

    public void driveRobotRelative(double y, double x, double rx) {
        // Drive robot from the robot frame of reference (robot relative).
        // Positive values of y move forward, positive values of x strafe to the right, positive values of rx rotate clockwise.

        double speedLimiter = 0.4; // reduces movement speed to a specified % of maximum (default = 1.0). Used for outreach events.

        // Normalize motor power. This ensures all the powers are scaled proportionally and remain in the range of [-1.0, 1.0].
        double scaleFactor = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        // Calculate the power needed for each wheel based on the amount of forward (y), strafe (x), and turn (rx)
        // See mecanum drive reference: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2234/13280/3209-0001-0007-Product-Insight-2__06708__33265.1725633323.png?c=1
        double frontLeftPower = ((y + x + rx) / scaleFactor) * speedLimiter;
        double backLeftPower = ((y - x + rx) / scaleFactor) * speedLimiter;
        double frontRightPower = ((y - x - rx) / scaleFactor) * speedLimiter;
        double backRightPower = ((y + x - rx) / scaleFactor) * speedLimiter;

        // Set scaled motor powers (with limiter).
        frontLeftDrive.setPower(squarePower(frontLeftPower));
        backLeftDrive.setPower(squarePower(backLeftPower));
        frontRightDrive.setPower(squarePower(frontRightPower));
        backRightDrive.setPower(squarePower(backRightPower));

        telemetry.addData("\nmaxPower", scaleFactor);
        telemetry.addData("frontLeftPower","%4.2f", (frontLeftPower / scaleFactor));
        telemetry.addData("backLeftPower","%4.2f", (backLeftPower / scaleFactor));
        telemetry.addData("frontRightPower","%4.2ff", (frontRightPower / scaleFactor));
        telemetry.addData("backRightPower","%4.2f", (backRightPower / scaleFactor));
        telemetry.update();
    }

    private static double squarePower(double power) {
        // Function to square motor power to allow for better micro control.  Returns a double.
        return power * Math.abs(power);  // square magnitude of input while maintaining the sign
    }
}