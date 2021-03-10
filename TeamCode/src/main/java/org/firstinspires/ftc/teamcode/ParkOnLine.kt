package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Park on line")
class ParkOnLine : LinearOpMode() {
    override fun runOpMode() {
        val robot = RobotHardware()
        robot.init(hardwareMap, "")
        waitForStart()
        robot.chassis(doubleArrayOf(-1.0, 0.0, 0.0))
        sleepInches(35.0)
        robot.brake()
        sleep(0)
        robot.startFlywheels()
        robot.startIntake()
        sleep(500)
        robot.fire()
        sleep(500)
        robot.fire()
        sleep(500)
        robot.fire()
        sleep(500)
        robot.stopFlywheels()
        robot.stopIntake()
        robot.chassis(doubleArrayOf(-1.0, 0.0, 0.0))
        sleepInches(10.0)
        robot.brake()
    }

    private fun sleepInches(inches: Double) {
        sleep((inches / (RobotHardware.INCHES_PER_SECOND / 1000)).toLong())
    }

}