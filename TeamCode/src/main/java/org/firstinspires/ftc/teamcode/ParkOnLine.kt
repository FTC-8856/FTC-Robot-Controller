package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Park on line")
class ParkOnLine : LinearOpMode() {
    private var robot: RobotHardware? = null
    override fun runOpMode() {
        robot = RobotHardware()
        robot!!.init(hardwareMap, "")
        waitForStart()
        robot!!.driveFor(90.0, doubleArrayOf(0.0, 0.0, 0.0))
    }

}