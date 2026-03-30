package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumDrive {

    // Declare motor and IMU objects (and make them private to prevent external access)
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private IMU imu;

    //GoBildaPinpointDriver pinpoint;

    private ElapsedTime runtime = new ElapsedTime();

    // Declare drive constants (and make public if you want to use by the calling OpMode).
    // Calculate the COUNTS_PER_INCH for your specific encoder. Check the vendor website to determine the encoder resolution for you particular motor.
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    private static final double COUNTS_PER_REV = 28;  // PPR REV 20:1 (5:1 + 4:1) = 28 * (5.23 * 3.67) = 537.4; goBilda 312RPM Yellow Jacket motor = 537.7
    private static final double DRIVE_GEAR_REDUCTION = 5.23 * 3.67;  // external drive/motor gearing or 1.0 if none.
    private static final double WHEEL_DIAMETER_MM = 75; // REV = 75mm; goBuilda = 104mm
    private static final double COUNTS_PER_MM = (COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);
    //private static final double TICKS_PER_IN = TICKS_PER_MM * 25.4;
    public static final double speedLimiter = 1.0;  // speedLimiter reduces movement speed to a specified % of maximum (1.0). Used for outreach events.

    public void init(HardwareMap hwMap) {
    //public void init(HardwareMap hwMap, boolean isGoBildaPinPointIMU) {

        // Initialize the hardware variables. Note that the strings specified here as parameters must match the names assigned in the robot controller configuration.
        frontLeftDrive = hwMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hwMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hwMap.get(DcMotor.class, "back_right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // TODO: Determine which motor pair needs to be reversed (LEFT or RIGHT).
        //frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        //backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set to RUN_USING_ENCODER mode
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");
        // TODO: Change the the following to match the controller Hub orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void setMotorModes(DcMotor.RunMode mode) {
    // Method to set all motor run modes
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return imu.getRobotYawPitchRollAngles();
    }

    public void driveFieldRelative(double forward, double strafe, double turn) {
    // Method to drive robot from the field frame of reference (field relative).

        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        // Second, turn angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and strafe amounts
        driveRobotRelative(newForward, newStrafe, turn);
    }

    public void driveRobotRelative(double y, double x, double rx) {
    // Method to drive robot from the robot frame of reference (robot relative).
    // Positive values of y move forward, positive values of x strafe to the right, positive values of rx rotate clockwise.
        x *= 1.1;   // counteract imperfect strafing

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
    }

    private static double squarePower(double power) {
    // Method to square motor power to allow for better micro control.  Returns a double.
        return power * Math.abs(power);  // square magnitude of input while maintaining the sign
    }

    public int getCurrentPosition() {
        return frontLeftDrive.getCurrentPosition();
    }

    public void driveEncoder(double speed, double distance, DistanceUnit distanceUnit) {
    /*
     This function uses the FTC SDK DistanceUnits class, which allows us to accept different input
     units depending on the user's preference.

     To use, put a double and a DistanceUnit as parameters in a function and then
     call distanceUnit.toMm(distance). This will return the number of millimeters that are equivalent
     to whatever distance in the unit specified. We are working in millimeters for this, so that's the
     unit we request from distanceUnit. But if we want to use inches in our function, we could
     use distanceUnit.toInches(distance) instead.
    */
        // Determine new target position, and pass to motor controller
        int targetPosition = (int)(distanceUnit.toMm(distance) * COUNTS_PER_MM);
        int frontLeftTarget = frontLeftDrive.getCurrentPosition() + targetPosition;
        int backLeftTarget = backLeftDrive.getCurrentPosition() + targetPosition;
        int frontRightTarget = frontRightDrive.getCurrentPosition() + targetPosition;
        int backRightTarget = backRightDrive.getCurrentPosition() + targetPosition;

        // Set target then set RUN_TO_POSITION
        frontLeftDrive.setTargetPosition(frontLeftTarget);
        backLeftDrive.setTargetPosition(backLeftTarget);
        frontRightDrive.setTargetPosition(frontRightTarget);
        backRightDrive.setTargetPosition(backRightTarget);
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        driveRobotRelative(Math.abs(speed), 0, 0);  // speed must be positive for RUN_TO_POSITION to work correctly
    }

    public void stopRobot() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}