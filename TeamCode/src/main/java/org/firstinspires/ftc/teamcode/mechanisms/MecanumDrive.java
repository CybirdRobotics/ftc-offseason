package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {

    // Declare motor and IMU objects (and make them private to prevent external access)
    private DcMotor leftFrontDrive, leftRearDrive, rightFrontDrive, rightRearDrive;
    private IMU imu;
    private Telemetry telemetry;    // allow for telemetry calls within this class
    private GoBildaPinpointDriver pinpoint;
    private ElapsedTime runtime = new ElapsedTime();

    // Declare drive constants (and make public if you want to use by the calling OpMode).
    // TODO: Calculate the TICKS_PER_INCH for your specific encoder. Check the vendor website to determine the encoder resolution for you particular motor.
    private static final double TICKS_PER_REV = 28;  // PPR REV 20:1 (5:1 + 4:1) = 28 * (5.23 * 3.67) = 537.4
    private static final double MOTOR_GEAR_RATIO = 5.23 * 3.67;  // REV motor gearing cartridges
    private static final double ENCODER_RESOLUTION = TICKS_PER_REV * MOTOR_GEAR_RATIO;    // REV 20:1 (5:1 + 4:1) = 28 * (5.23 * 3.67) = 537.4; goBilda 312RPM Yellow Jacket motor = 537.7 PPR
    private static final double DRIVE_GEAR_RATIO = 1.0; // external drive gear ratio or 1.0 if none
    private static final double WHEEL_DIAMETER_MM = 75; // REV = 75mm; goBuilda = 104mm
    private static final double TICKS_PER_MM = (ENCODER_RESOLUTION * DRIVE_GEAR_RATIO) / (WHEEL_DIAMETER_MM * Math.PI);

    // These constants define the desired driving/control characteristics.
    // They can/should be tweaked to for the specific robot drivetrain.
    public static final double SPEED_LIMITER = 0.8; // speedLimiter reduces movement speed to a specified % of maximum (1.0). Used for outreach events.
    static final double HEADING_THRESHOLD = 2.0 ;   // How close must the heading get to the target before moving to next step.
                                                    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.

    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omnidirectional wheels).
    static final double kpTurn = 0.02;     // Larger is more responsive, but also less stable.

    // Define a constructor that accepts the calling OpMode's telemetry instance
    public MecanumDrive(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hwMap, boolean isPinPointIMU) {

        // Get a reference to and initialize the IMU using the built-in REV Control Hub IMU.
        imu = hwMap.get(IMU.class, "imu");

        // TODO: Change the the following to match the controller Hub orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Get references to and initialize the motors. Note that the strings specified here as parameters must match the names assigned in the robot controller configuration.
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hwMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hwMap.get(DcMotor.class, "right_rear_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // TODO: Determine which motor pair needs to be reversed (LEFT or RIGHT).
        //frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        //backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set to RUN_USING_ENCODER mode
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        if (isPinPointIMU) {
            // Get a reference to and set up the goBilda Pinpoint sensor
            pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
            configurePinpoint();    // configure the goBilda Pinpoint sensor
        }
    }

    public void setMotorModes(DcMotor.RunMode mode) {
    // Method to set all motor run modes
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftRearDrive.setMode(mode);
        rightRearDrive.setMode(mode);
    }

    public void configurePinpoint() {
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-3.875, 0, DistanceUnit.INCH); // refer to goBilda 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }

    public void setPinpointPosition(DistanceUnit distanceUnit, double x, double y, AngleUnit angleUnit, double heading) {
        // set the initial location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(distanceUnit, x, y, angleUnit, heading));
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    public int getCurrentPosition() {
        return leftFrontDrive.getCurrentPosition();
    }

    public void driveFieldRelative(double forward, double strafe, double turn) {
    // Method to drive robot from the field frame of reference (field relative).

        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        // Second, turn angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - getHeading(AngleUnit.RADIANS));

        // Third, convert back to Cartesian coordinates
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and strafe amounts
        driveRobotRelative(newForward, newStrafe, turn);
    }

    public void driveRobotRelative(double forward, double strafe, double turn) {

        // Method to drive robot from the robot frame of reference (robot relative).
        // Positive values of y move forward, positive values of x strafe to the right, positive values of rx rotate clockwise.
        //x *= 1.1;   // counteract imperfect strafing (changed to pass in via OpMode)

        // Calculate the raw power for each wheel based on the amount of forward (y), strafe (x), and turn (rx) inputs
        // See mecanum drive reference: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2234/13280/3209-0001-0007-Product-Insight-2__06708__33265.1725633323.png?c=1
        double leftFrontPower = (forward + strafe + turn);
        double leftRearPower = (forward - strafe + turn);
        double rightFrontPower = (forward - strafe - turn);
        double rightRearPower = (forward + strafe - turn);

        // Normalize motor power. This ensures all the powers are scaled proportionally and remain in the range of [-1.0, 1.0].
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0 ) {
            leftFrontPower /= max;
            leftRearPower /= max;
            rightFrontPower /= max;
            rightRearPower /= max;
        }

        // Set scaled motor powers (with limiter).
        leftFrontDrive.setPower(squarePower(leftFrontPower * SPEED_LIMITER));
        leftRearDrive.setPower(squarePower(leftRearPower * SPEED_LIMITER));
        rightFrontDrive.setPower(squarePower(rightFrontPower * SPEED_LIMITER));
        rightRearDrive.setPower(squarePower(rightRearPower * SPEED_LIMITER));
    }

    private static double squarePower(double power) {
    // Method to square motor power to allow for better micro control.  Returns a double.
        return power * Math.abs(power);  // square magnitude of input while maintaining the sign
    }

    public void encoderDrive(double speed, double distance, DistanceUnit distanceUnit) {
    /*
     This function uses the FTC SDK DistanceUnits class, which accepts different input units depending on the user's preference.

     To use, put a double and a DistanceUnit as parameters in a function and then call distanceUnit.toMm(distance). This will
     return the number of millimeters that are equivalent to whatever distance in the unit specified. We are working in millimeters
     for this, so that's the unit we request from distanceUnit. But if we want to use inches in our function, we could
     use distanceUnit.toInches(distance) instead.
    */
        // Determine new target position. This method assumes that each movement is relative to the robots last position.
        int targetPosition = (int)(distanceUnit.toMm(distance) * TICKS_PER_MM);
        int frontLeftTarget = leftFrontDrive.getCurrentPosition() + targetPosition;
        int backLeftTarget = leftRearDrive.getCurrentPosition() + targetPosition;
        int frontRightTarget = rightFrontDrive.getCurrentPosition() + targetPosition;
        int backRightTarget = rightRearDrive.getCurrentPosition() + targetPosition;

        // Set target in motor controller then set RUN_TO_POSITION
        leftFrontDrive.setTargetPosition(frontLeftTarget);
        leftRearDrive.setTargetPosition(backLeftTarget);
        rightFrontDrive.setTargetPosition(frontRightTarget);
        rightRearDrive.setTargetPosition(backRightTarget);
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        driveRobotRelative(Math.abs(speed), 0, 0);  // speed must be positive for RUN_TO_POSITION to work correctly
    }

    public void turnToHeading(double targetHeading, double speed) {
    // Method to turn the robot to a desired heading

        double error = AngleUnit.normalizeDegrees(targetHeading - getHeading(AngleUnit.DEGREES)); // initial call to seed current error

        while (Math.abs(error) > HEADING_THRESHOLD) {
            error = AngleUnit.normalizeDegrees(targetHeading - getHeading(AngleUnit.DEGREES));
            double turnSpeed = error * kpTurn;  // calculate motor power proportional to the error

            driveRobotRelative(0, 0, Range.clip(turnSpeed, -speed, speed)); // Turn robot.  Limit the result to +/- max input speed.
        }

        stopRobot();    // stop movement once target is reached.
    }

    public boolean isBusy() {
    // Method to check for active drive motor(s).  Returns a boolean.
        return (leftFrontDrive.isBusy() || leftRearDrive.isBusy() || rightFrontDrive.isBusy() || rightRearDrive.isBusy());
    }

    public void stopRobot() {
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
    }
}