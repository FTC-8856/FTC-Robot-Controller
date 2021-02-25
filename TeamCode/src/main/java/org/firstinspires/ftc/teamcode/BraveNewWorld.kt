package org.firstinspires.ftc.teamcode

import android.graphics.Color
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position

@TeleOp(name = "Brave New World", group = "Pushbot")
open class BraveNewWorld : OpMode() {
    val robot = RobotHardware() // use the class created to define a Pushbot's hardware
    private var lastArm = 0.0
    open fun extendInit() {}
    open fun extendLoop() {}
    open fun extendStop() {}

    /*
     * Code to run ONCE when the driver hits INIT
     */
    override fun init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, "imu")
        extendInit()
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    override fun init_loop() {
        telemetry.addData("Brave New World", "Groovy")
        telemetry.addData("Version", "0.21")
        telemetry.addData("Accelerometer", robot.imu?.isAccelerometerCalibrated)
        telemetry.addData("Gyro", robot.imu?.isGyroCalibrated)
        telemetry.addData("Magnetometer", robot.imu?.isMagnetometerCalibrated)
        telemetry.addData("Calib. Status", robot.imu?.calibrationStatus.toString())
        telemetry.update()
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    override fun start() {
        robot.startIMU(Position(
                DistanceUnit.METER,
                0.0, 0.0, 0.0 // <--- starting position
                , 0
        ))
    }

    override fun loop() {
        robot.hardwareLoop()
        robot.chassis(doubleArrayOf(gamepad1.right_stick_y.toDouble(), gamepad1.right_stick_x.toDouble(), gamepad1.left_stick_x.toDouble()))
        val arm = gamepad2.left_stick_y.toDouble()
        if (arm != lastArm) {
            robot.armPower(arm)
            lastArm = arm
        }
        if (gamepad2.right_bumper) {
            robot.openClaw()
        } else if (gamepad2.right_trigger > 0.1) {
            robot.closeClaw()
        }
        if (gamepad2.back) {
            robot.armAtStartup()
        }
        if (gamepad2.right_stick_button) {
            if (robot.areFlywheelsRunning()) {
                robot.stopFlywheels()
            } else {
                robot.startFlywheels()
            }
        }
        if (gamepad2.dpad_down) {
            robot.decrementFirePosition()
        }
        if (gamepad2.dpad_up) {
            robot.incrementFirePosition()
        }
        if (gamepad2.left_stick_button) {
            robot.fire()
        }
        if (gamepad2.a) {
            robot.startIntake()
        }
        if (gamepad2.b) {
            robot.reverseIntake()
        }
        if (gamepad2.x) {
            robot.stopIntake()
        }
        extendLoop()
        telemetry.addData("fwd/bkwd", "%.2f", gamepad1.right_stick_y)
        telemetry.addData("strafe", "%.2f", gamepad1.right_stick_x)
        telemetry.addData("turn", "%.2f\n------------", gamepad1.left_stick_x)
        telemetry.addData("Rot", "(%.2f, %.2f, %.2f)", robot.rot?.thirdAngle, robot.rot?.secondAngle, robot.rot?.firstAngle)
        telemetry.addData("Pos", "(%.2fm, %.2fm, %.2fm)", robot.pos?.x, robot.pos?.y, robot.pos?.z)
        telemetry.addData("Fire position", "%s", robot.firePos)
        telemetry.addData("Arm position", "%.2f", robot.wobble?.position)
        telemetry.update()
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    override fun stop() {
        robot.hardwareStop()
        extendStop()
        telemetry.addData("Exit", "Goodest Good Job!")
        telemetry.update()
    }
}