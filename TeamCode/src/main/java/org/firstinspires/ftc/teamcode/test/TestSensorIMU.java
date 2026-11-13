package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Sensors;

@TeleOp(name="Test Sensors (IMU)", group = "Test")
//@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class TestSensorIMU extends OpMode {

    // Create an instance of the Sensors object to be used to access robot sensors.
    Sensors sensors = new Sensors(telemetry);

    @Override
    public void init() {
        sensors.init(hardwareMap);
    }

    @Override
    public void loop() {

        telemetry.addLine("Press A (X) to reset YAW.\n");

        // When gamepad1.a is pressed, reset the Yaw to 0 based on the orientation relative to the robot's position
        if (gamepad1.a) {
            sensors.resetHeading();
        }

        telemetry.addData("Heading (degrees)", sensors.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Heading (radians)", sensors.getHeading(AngleUnit.RADIANS));
    }
}