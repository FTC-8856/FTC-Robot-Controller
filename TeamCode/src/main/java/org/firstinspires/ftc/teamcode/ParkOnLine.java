package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@SuppressWarnings("unused")
@Autonomous(name = "Park on line")
class ParkOnLine extends LinearOpMode {


    @Override
    public final void runOpMode() {
        final RobotHardware robot = new RobotHardware();
        robot.init(this.hardwareMap, "");
        this.waitForStart();
        robot.driveFor(90.0, new double[]{0.0, 0.0, 0.0});

    }
}
