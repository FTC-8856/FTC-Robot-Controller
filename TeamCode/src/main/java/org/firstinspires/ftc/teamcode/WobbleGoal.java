package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Wobble Goal")
public class WobbleGoal extends LinearOpMode {
    private static final Double INCHES_PER_SECOND = 52.5;
    RobotHardware robot;

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
        robot.closeClaw();
        drive_for(88.0);
        robot.armPower(-1.0);
        robot.openClaw();
        robot.armPower(1.0);
    }

    private void drive_for(Double inches) {
        robot.chassis(-1.0, 0, 0);
        waitFor((long) (1000 * inches / INCHES_PER_SECOND));
    }
}
