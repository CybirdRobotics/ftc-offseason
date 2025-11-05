package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
//@Disabled // comment this out to add to the OpMode list on the Driver Hub
public class HelloRobot extends OpMode {
    // this is a commen
    //comment aslp
    // another comment

    /* comment
    comment

     */

    @Override
    public void init() {    // this is the init method
    }

    @Override
    public void loop() {
        telemetry.addLine("Hello Robot");
        telemetry.update();
    }
}
