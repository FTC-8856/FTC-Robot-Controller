package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@SuppressWarnings("unused")
@TeleOp(name = "Insecticide", group = "Bug Fix the Bug")
class Insecticide extends OpMode {
    @NonNull
    private final
    RobotHardware robot = new RobotHardware(); // use the class created to define a Pushbot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public final void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        this.robot.init(this.hardwareMap, "");
    }

    @Override
    public final void loop() {
        this.robot.hardwareLoop();
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
    public final void stop() {
        this.robot.hardwareStop();
    }
}
