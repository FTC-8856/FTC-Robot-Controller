package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector

@Autonomous(name = "Wobble Goal")
class WobbleGoal : LinearOpMode() {
    private var robot: RobotHardware = RobotHardware()
    private var vuforia: VuforiaLocalizer? = null
    private var tfod: TFObjectDetector? = null

    override fun runOpMode() {
        robot.init(hardwareMap, "")
        initVuforia()
        initTfod()
        telemetry.addData("Waiting", null)
        val recognized = getRecognition()
        waitForStart()
        when (recognized) {
            0 -> zero()
            1 -> one()
            4 -> four()
        }
    }

    private fun getRecognition() : Int {
        val recognitions = tfod!!.recognitions
        if (recognitions.isEmpty()) {
            tfod!!.shutdown()

            telemetry.addData("Rings:", "Zero (no recognitions found)")
            telemetry.update()

            return 0
        } else {
            var oneConfidence = 0.0
            var fourConfidence = 0.0
            for (recognition in recognitions) {
                val confidence = recognition.confidence.toDouble()
                if (recognition.label == LABEL_SECOND_ELEMENT && confidence > oneConfidence) {
                    oneConfidence = confidence
                }
                if (recognition.label == LABEL_FIRST_ELEMENT && confidence > fourConfidence) {
                    fourConfidence = confidence
                }
            }
            tfod!!.shutdown()
            return if (fourConfidence < oneConfidence) {
                telemetry.addData("Rings:", "One")
                telemetry.update()

                1
            } else {
                telemetry.addData("Rings:", "Four")
                telemetry.update()

                4
            }
        }
    }

    private fun zero() {
        val forwardInches = 70.0
        val rightInches = 40.0

        driveForward(inches = forwardInches)
        turnRight(inches = rightInches)
        drop()
    }

    private fun one() {
        val forwardInches = 100.0

        driveForward(inches = forwardInches)
        drop()
    }

    private fun four() {
        val leftInches = 36.0
        val forwardInches = 110.0
        val rightInches = 48.0

        driveLeft(inches = leftInches)
        driveForward(inches = forwardInches)
        driveRight(inches = rightInches)
        drop()
    }

    private fun driveForward(power: Double = 1.0, inches: Double) {
        robot.chassis(doubleArrayOf(power, 0.0, 0.0))
        sleepInches(inches)
        robot.brake()
    }

    private fun driveLeft(power: Double = 1.0, inches: Double) {
        robot.chassis(doubleArrayOf(0.0, -1.0 * power, 0.0))
        sleepInches(inches)
        robot.brake()
    }

    private fun driveRight(power: Double = 1.0, inches: Double) {
        robot.chassis(doubleArrayOf(0.0, power, 0.0))
        sleepInches(inches)
        robot.brake()
    }

    private fun turnRight(power: Double = 1.0, inches: Double) {
        robot.chassis(doubleArrayOf(0.0, 0.0, power))
        sleepInches(inches)
        robot.brake()
    }


    private fun sleepInches(inches: Double) {
        sleep((inches / (RobotHardware.INCHES_PER_SECOND / 1000)).toLong())
    }

    private fun drop() {
        robot.openClamp()
        robot.startIntake()
        sleep(3000)
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private fun initVuforia() {
        val parameters = VuforiaLocalizer.Parameters()
        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraName = robot.webcam
        vuforia = ClassFactory.getInstance().createVuforia(parameters)
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private fun initTfod() {
        val tfodMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.packageName)
        val tfodParameters = TFObjectDetector.Parameters(tfodMonitorViewId)
        tfodParameters.minResultConfidence = 0.6f
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)
        tfod!!.apply {
            this.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT)
            this.setZoom(ZOOM, 16.0 / 9.0)
            this.activate()
        }
    }

    companion object {
        private const val TFOD_MODEL_ASSET = "UltimateGoal.tflite"
        private const val LABEL_FIRST_ELEMENT = "Quad"
        private const val LABEL_SECOND_ELEMENT = "Single"
        private const val VUFORIA_KEY = "AUAWFpz/////AAABmVwtdR5e/EuYge3oIRmKWEsX1ls1SEgysmAKbYVf8clIR74ciZ7+ucQX+zdIHUmJKWSbIBsZsJpXPgDONKCMYc2Ybsg4Wy7362azDSVNBmZEtKSeEVFG7d2NKTTsiJgX3KkQE75T0TYXcaxc5A/CIgQ63d9Xv/vmN5ytCt4Lkur9sB3ZyTnSUNbn3b3e0H+tt0mHYeksYP/+CRL7WlOSXyz02VRf7AqlzT62V7VXSbkWzWx3EwC7y5Oe7vQ/MLAO+e7fgOaybwZyO4bFVreLY/2pojB2ciMg2Sb8eyIjsLvMXAfPCOwpY3OkWsvFWuZxR5gdfvNGC0q57H3xpFHpXHv9sG1KPJTxL9MROn3KfNtd"
        private const val ZOOM = 1.9
    }
}