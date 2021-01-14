package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@SuppressWarnings("unused")
@Autonomous(name = "Wobble Goal")
class WobbleGoal extends LinearOpMode {

    private static void zero(final RobotHardware robot) {
        robot.closeClaw();
        robot.driveFor(88.0, new double[]{0.0, 0.0, 0.0});
        robot.armPower(-1.0);
        robot.openClaw();
        robot.armPower(1.0);
    }

    @Override
    public final void runOpMode() {
        final RobotHardware robot = new RobotHardware();
        robot.init(this.hardwareMap, "");
        this.waitForStart();
        WobbleGoal.zero(robot);
    }
}
