package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.tensorflow.lite.Interpreter
import java.io.File

@Autonomous(name = "Greg Brain") // Yeeeees we are getting the brain
class GregBrain : OpMode() {
    private val gametime = ElapsedTime()
    private val robot = RobotHardware()
    private var brain: Interpreter? = null
    override fun init() {
        val tflite_file = File("/FIRST/8856tensorflow/greg.tflite")
        brain = Interpreter(tflite_file)
        robot.init(hardwareMap, "imu")
    }

    override fun start() {
        robot.startIMU(Position(
                DistanceUnit.METER,
                0.0, 0.0, 0.0 // <--- starting position
                , 0
        ))
        gametime.reset()
    }

    override fun loop() {
        robot.hardwareLoop()
        val input_layer = robot.pos?.x?.toFloat()?.let {
            robot.rot?.let { it1 ->
                floatArrayOf(
                    gametime.seconds().toFloat() / 30 // 30 second autonomous period
                    , it
                    , robot.pos!!.y.toFloat()
                    , it1.thirdAngle // Heading
            )
            }
        }
        val output_layer = FloatArray(4) // Basic Joystick controls for now
        if (input_layer != null) {
            brain!!.run(input_layer, output_layer)
        } // activate the  t h i n k i n g
        robot.chassis(doubleArrayOf(output_layer[0].toDouble(), output_layer[1].toDouble(), output_layer[2].toDouble()))
        robot.performAction(decodeFloat(output_layer[3]))
    }

    override fun stop() {
        brain!!.close()
        robot.hardwareStop()
    }

    private fun decodeFloat(f: Float): Action {
        val f_11 = f / 11.0
        if (f_11 >= 0 && f_11 < 1) {
            return Action.OPEN_CLAW
        }
        if (f_11 >= 1 && f_11 < 2) {
            return Action.CLOSE_CLAW
        }
        if (f_11 >= 2 && f_11 < 3) {
            return Action.RETRACT_ARM
        }
        if (f_11 >= 3 && f_11 < 4) {
            return Action.EXTEND_ARM
        }
        if (f_11 >= 4 && f_11 < 5) {
            return Action.START_INTAKE
        }
        if (f_11 >= 5 && f_11 < 6) {
            return Action.STOP_INTAKE
        }
        if (f_11 >= 6 && f_11 < 7) {
            return Action.REVERSE_INTAKE
        }
        if (f_11 >= 7 && f_11 < 8) {
            return Action.FIRE_LOW
        }
        if (f_11 >= 8 && f_11 < 9) {
            return Action.FIRE_MID
        }
        return if (f_11 >= 9 && f_11 < 10) {
            Action.FIRE_HIGH
        } else Action.NONE
    }
}