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
        telemetry.addData("Waiting", null);
        waitForStart()
        goTo()
    }

    private fun goTo() {
        if (tfod != null) {
            val recognitions = tfod!!.recognitions
            if (recognitions.isEmpty()) {
                tfod!!.shutdown()
                zero()
            } else {
                var oneConfidence = 0.0
                var fourConfidence = 0.0
                for (recognition in recognitions) {
                    if (recognition.label == LABEL_SECOND_ELEMENT) {
                        oneConfidence = recognition.confidence.toDouble()
                    } else if (recognition.label == LABEL_FIRST_ELEMENT) {
                        fourConfidence = recognition.confidence.toDouble()
                    }
                }
                if (fourConfidence > oneConfidence) {
                    tfod!!.shutdown()
                    four()
                } else {
                    tfod!!.shutdown()
                    one()
                }
            }
        } else {
            telemetry.addData("no TensorFlow object detector found", null)
        }
    }

    private fun zero() {
        val forwardInches = 100.0

        telemetry.addData("Rings:", "Zero (no recognitions found)")
        telemetry.update()

        driveForward(forwardInches)
        drop()
    }

    private fun one() {
        val leftInches = 48.0
        val forwardInches = 100.0

        telemetry.addData("Rings:", "One")
        telemetry.update()

        driveLeft(leftInches)
        driveForward(forwardInches)
        drop()
    }

    private fun four() {
        val sideInches = 36.0
        val forwardInches = 180.0

        telemetry.addData("Rings:", "Four")
        telemetry.update()

        driveLeft(sideInches)
        driveForward(forwardInches)
        driveRight(sideInches)
        drop()
    }

    private fun driveForward(inches: Double) {
        robot.chassis(doubleArrayOf(1.0, 0.0, 0.0))
        sleep_inches(inches)
        robot.brake()
    }

    private fun driveLeft(inches: Double) {
        robot.chassis(doubleArrayOf(0.0, -1.0, 0.0))
        sleep_inches(inches)
        robot.brake()
    }

    private fun driveRight(inches: Double) {
        robot.chassis(doubleArrayOf(0.0, 1.0, 0.0))
        sleep_inches(inches)
        robot.brake()
    }

    private fun turnLeft(inches: Double) {
        robot.chassis(doubleArrayOf(0.0, 0.0, -1.0))
        sleep_inches(inches)
        robot.brake()
    }

    private fun turnRight(inches: Double) {
        robot.chassis(doubleArrayOf(0.0, 0.0, 1.0))
        sleep_inches(inches)
        robot.brake()
    }


    private fun sleep_inches(inches: Double) {
        sleep((inches / (RobotHardware.INCHES_PER_SECOND / 1000)).toLong())
    }

    private fun drop() {
        robot.armPower(1.0)
        sleep(1000)
        robot.openClaw()
        sleep(1000)
        robot.armPower(-1.0)
        robot.closeClaw()
        sleep(1000)
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
        val zoom = 1.8

        val tfodMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.packageName)
        val tfodParameters = TFObjectDetector.Parameters(tfodMonitorViewId)
        tfodParameters.minResultConfidence = 0.6f
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)
        tfod!!.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT)
        tfod!!.setZoom(zoom, 16.0 / 9.0)
        tfod!!.activate()
    }

    companion object {
        private const val TFOD_MODEL_ASSET = "UltimateGoal.tflite"
        private const val LABEL_FIRST_ELEMENT = "Quad"
        private const val LABEL_SECOND_ELEMENT = "Single"
        private const val VUFORIA_KEY = "AUAWFpz/////AAABmVwtdR5e/EuYge3oIRmKWEsX1ls1SEgysmAKbYVf8clIR74ciZ7+ucQX+zdIHUmJKWSbIBsZsJpXPgDONKCMYc2Ybsg4Wy7362azDSVNBmZEtKSeEVFG7d2NKTTsiJgX3KkQE75T0TYXcaxc5A/CIgQ63d9Xv/vmN5ytCt4Lkur9sB3ZyTnSUNbn3b3e0H+tt0mHYeksYP/+CRL7WlOSXyz02VRf7AqlzT62V7VXSbkWzWx3EwC7y5Oe7vQ/MLAO+e7fgOaybwZyO4bFVreLY/2pojB2ciMg2Sb8eyIjsLvMXAfPCOwpY3OkWsvFWuZxR5gdfvNGC0q57H3xpFHpXHv9sG1KPJTxL9MROn3KfNtd"
    }
}