/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Greg")
class Greg : LinearOpMode() {
    /**
     * [.vuforia] is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private var vuforia: VuforiaLocalizer? = null

    /**
     * [.tfod] is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private var tfod: TFObjectDetector? = null

    private var robot: RobotHardware = RobotHardware()

    override fun runOpMode() {
        robot.init(hardwareMap, "")
        initVuforia()
        initTfod()


        forward(2.0)
        tfod!!.activate()
        val numberRings = getNumberRings()

        tfod!!.shutdown()

        telemetry.addData("Waiting", null)
        telemetry.update()

        waitForStart()

        when (numberRings) {
            0 -> goToZero()
            1 -> goToOne()
            4 -> goToFour()
        }
    }

    private fun goToZero() {
        forward(40.0)
        drop()
    }

    private fun goToOne() {
        forward(40.0)
        drop()
    }

    private fun goToFour() {
        forward(40.0)
        drop()
    }

    private fun getNumberRings(): Int {
        var oneConfidence = 0.0
        var fourConfidence = 0.0

        var count = 0

        while (count < 100) {
            val recs = getRecognitions()
            if (recs[0] > oneConfidence) {
                oneConfidence = recs[0]
            }
            if (recs[1] > fourConfidence) {
                fourConfidence = recs[1]
            }

            count += 1
        }

        return if (oneConfidence == 0.0 && fourConfidence == 0.0) {
            0
        } else if (oneConfidence > fourConfidence) {
            1
        } else {
            4
        }
    }

    private fun getRecognitions(): DoubleArray {
        val recognitions = tfod!!.recognitions

        var oneConfidence = 0.0
        var fourConfidence = 0.0
        if (recognitions != null) {
            val one = recognitions.find { it.label == LABEL_SECOND_ELEMENT }?.confidence?.toDouble()
            if (one != null) {
                oneConfidence = one
            } else {
                telemetry.addData("Nothing labeled `Single` was found", null)
            }
            val four = recognitions.find { it.label == LABEL_FIRST_ELEMENT }?.confidence?.toDouble()
            if (four != null) {
                fourConfidence = four
            } else {
                telemetry.addData("Nothing labeled `Quad` was found", null)
            }

        } else {
            telemetry.addData("recognitions was null", null)
        }
        return doubleArrayOf(oneConfidence, fourConfidence)
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private fun initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        val parameters = VuforiaLocalizer.Parameters()
        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraName = hardwareMap.get(WebcamName::class.java, "gregcam")

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters)

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private fun initTfod() {
        val tfodMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.packageName)
        val tfodParameters = TFObjectDetector.Parameters(tfodMonitorViewId)
        tfodParameters.minResultConfidence = 0.8f
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)
        tfod!!.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT)
    }

    companion object {
        private const val TFOD_MODEL_ASSET = "UltimateGoal.tflite"
        private const val LABEL_FIRST_ELEMENT = "Quad"
        private const val LABEL_SECOND_ELEMENT = "Single"

        private const val VUFORIA_KEY = "AUAWFpz/////AAABmVwtdR5e/EuYge3oIRmKWEsX1ls1SEgysmAKbYVf8clIR74ciZ7+ucQX+zdIHUmJKWSbIBsZsJpXPgDONKCMYc2Ybsg4Wy7362azDSVNBmZEtKSeEVFG7d2NKTTsiJgX3KkQE75T0TYXcaxc5A/CIgQ63d9Xv/vmN5ytCt4Lkur9sB3ZyTnSUNbn3b3e0H+tt0mHYeksYP/+CRL7WlOSXyz02VRf7AqlzT62V7VXSbkWzWx3EwC7y5Oe7vQ/MLAO+e7fgOaybwZyO4bFVreLY/2pojB2ciMg2Sb8eyIjsLvMXAfPCOwpY3OkWsvFWuZxR5gdfvNGC0q57H3xpFHpXHv9sG1KPJTxL9MROn3KfNtd"
    }

    private fun forward(inches: Double) {
        robot.chassis(doubleArrayOf(1.0, 0.0, 0.0))
        sleepInches(inches)
        robot.brake()
    }

    private fun drop() {
        robot.openClamp()
        robot.startIntake()
        sleep(3000)
        robot.stopIntake()
        robot.closeClamp()
    }

    private fun sleepInches(inches: Double) {
        sleep((inches / (RobotHardware.INCHES_PER_SECOND / 1000)).toLong())
    }
}