/*
   From Coach Pratt "GoBILDAs DECODE Starter Robot is Good.  Let's Make it Great" YouTube video on improving programming techniques.
   https://youtu.be/zfPU8Vy9yiY?si=A0EJxVQ1khANI9vK
*/

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name="Mecanum Drive (Coach Pratt)", group = "OpMode")
//@Disabled
public class TeleOpExample extends OpMode {

    MecanumDrive drive = new MecanumDrive();    // create instance of MecanumDrive object (class)

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
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
    }
}
