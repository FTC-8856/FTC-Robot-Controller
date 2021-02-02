package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Park on line")
class ParkOnLine : LinearOpMode() {
    override fun runOpMode() {
        val robot = RobotHardware()
        robot.init(hardwareMap, "")
        waitForStart()
        robot.chassis(doubleArrayOf(0.0, 0.0, 0.0))
    }

}