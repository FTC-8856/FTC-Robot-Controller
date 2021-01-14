package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Insecticide", group = "Bug Fix the Bug")
public class Insecticide extends OpMode {
    @NonNull
    final
    RobotHardware robot = new RobotHardware(); // use the class created to define a Pushbot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, "");
    }

    @Override
    public void loop() {
        robot.hardwareLoop();
        /*
        if (gamepad1.a) {
            robot.set_backleft(1.0);
        }
        if (gamepad1.b) {
            robot.set_backright(1.0);
        }
        if (gamepad1.x) {
            robot.set_frontleft(1.0);
        }
        if (gamepad1.y) {
            robot.set_frontright(1.0);
        }
         */
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.hardwareStop();
    }
}
