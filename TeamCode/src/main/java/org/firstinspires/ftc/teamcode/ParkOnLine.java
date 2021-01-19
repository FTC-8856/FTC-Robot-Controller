package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@SuppressWarnings("unused")
@Autonomous(name = "Park on line")
public class ParkOnLine extends LinearOpMode {
    private static final Double INCHES_PER_SECOND = 52.5;
    RobotHardware robot;

    @Override
    public void runOpMode() {
        robot = new RobotHardware();
        robot.init(hardwareMap, "");
        waitForStart();
        robot.driveFor(90.0);
    }

}
