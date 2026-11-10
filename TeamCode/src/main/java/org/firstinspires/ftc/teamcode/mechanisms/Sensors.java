// Based on IMU/Gyro Beginners Guide (Learn Java for FTC) from Brogan Pratt YouTube channel.
// https://youtu.be/8mB7x6SNUKo?si=lkZ1lWrzfE79VjPO

package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Sensors {

    private IMU imu;    // declare IMU object (and make it private to prevent external access)
    private Telemetry telemetry;

    // Define a constructor that accepts the OpMode's telemetry instance
    public Sensors(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hwMap) {

        imu = hwMap.get(IMU.class, "imu");  // initialize the IMU using the built-in REV Control Hub IMU

        // TODO: Change the the following to match the controller Hub orientation on your robot
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public double getHeading(AngleUnit angleUnit) { // AngleUnit allows specification of AngleUnit.DEGREES or AngleUnit.RADIANS
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    public double normalizeHeading(double degrees) {
        if (degrees > 180) {    // normalize the target heading to stay within the IMU sensor range [-180, 180]
            degrees -= 360;
        } else if (degrees <= -180) {
            degrees += 360;
        }

        return degrees;
    }
}
