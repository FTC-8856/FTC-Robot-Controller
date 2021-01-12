package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Park on line")
public class ParkOnLine extends LinearOpMode {
    RobotHardware robot;
    private static final Double INCHES_PER_SECOND = 52.5;

    private void waitFor(long ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
        }
        robot.chassis(0, 0, 0);
    }

    @Override
    public void runOpMode() {
        robot = new RobotHardware();
        robot.init(hardwareMap, "");
        waitForStart();
        drive_for(90.0);
    }

    private void drive_for(Double inches) {
        robot.chassis(-1.0, 0, 0);
        waitFor((long)(1000 * inches / INCHES_PER_SECOND));
    }
}
