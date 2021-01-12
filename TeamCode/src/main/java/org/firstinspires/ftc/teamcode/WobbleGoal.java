package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Wobble Goal")
public class WobbleGoal extends LinearOpMode {
    RobotHardware robot;
    private static final Double INCHES_PER_SECOND = 52.5;

    private void waitFor(long ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
        }
        robot.chassis(0, 0.2, 0);
    }

    @Override
    public void runOpMode() {
        robot = new RobotHardware();
        robot.init(hardwareMap, "");
        waitForStart();
        zero();
    }

    private void zero() {
        robot.close_claw();
        drive_for(88.0);
        robot.arm_power(-1.0);
        robot.open_claw();
        robot.arm_power(1.0);
    }

    private void drive_for(Double inches) {
        robot.chassis(-1.0, 0, 0);
        waitFor((long)(1000 * inches / INCHES_PER_SECOND));
    }
}
