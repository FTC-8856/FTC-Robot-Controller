/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.Velocity
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import java.util.*

/**
 * This is NOT an opmode.
 */
class RobotHardware  /* Constructor */ {
    private val motorMap: MutableMap<DcMotor, Array<Double>> = HashMap()

    /* IMU public variables */
    private var isImuEnabled = false
    var imu: BNO055IMU? = null
        private set
    private var pos: Position? = null
        private set
    var rot: Orientation? = null
        private set
    private var intake: DcMotor? = null
    var wobble: Servo? = null
    private var wobbleFinger: Servo? = null
    var clamp: Servo? = null
    private var leftFlywheel: DcMotor? = null
    private var leftFlyMonitor: ExpansionHubMotor? = null
    private var rightFlywheel: DcMotor? = null
    private var rightFlyMonitor: ExpansionHubMotor? = null
    private var shooter: Servo? = null

    /* Initialize standard Hardware interfaces */
    fun init(ahwMap: HardwareMap, features: String) {
        // Save reference to Hardware map
        /* local OpMode members. */

        // Define and Initialize Motors
        /* Public OpMode members. */
        val frontleft = ahwMap.get(DcMotor::class.java, "frontleft")
        val frontright = ahwMap.get(DcMotor::class.java, "frontright")
        val backleft = ahwMap.get(DcMotor::class.java, "backleft")
        val backright = ahwMap.get(DcMotor::class.java, "backright")
        intake = ahwMap.get(DcMotor::class.java, "intake") // S c o o p s  the rings into the hopper to be shot at unsuspecting power shots and tower goals
        wobble = ahwMap.get(Servo::class.java, "wobble") // Wobble goal actuator arm rotator
        wobbleFinger = ahwMap.get(Servo::class.java, "wobbleFinger") // Wobble goal actuator arm "finger" grabber joint servo
        //greg = ahwMap.get(ColorSensor::class.java, "greg") // Greg is our premier color sensor.
        leftFlywheel = ahwMap.get(DcMotor::class.java, "leftflywheel")
        rightFlywheel = ahwMap.get(DcMotor::class.java, "rightflywheel")
        shooter = ahwMap.get(Servo::class.java, "shooter")
        clamp = ahwMap.get(Servo::class.java, "clamp")

        // Power Monitors (thanks OpenFTC)
        leftFlyMonitor = leftFlywheel as ExpansionHubMotor
        rightFlyMonitor = rightFlywheel as ExpansionHubMotor

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        frontright.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backleft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backright.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        if (features.contains("imu")) {
            isImuEnabled = true
            imu = ahwMap.get(BNO055IMU::class.java, "imu")

            // Create new IMU Parameters object.
            val imuParameters = BNO055IMU.Parameters()
            imuParameters.mode = BNO055IMU.SensorMode.IMU
            // Use degrees as angle unit.
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
            // Express acceleration as m/s^2.
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            imuParameters.calibrationDataFile = "GregIMUCalibration.json"
            //imuParameters.accelerationIntegrationAlgorithm = ; // We're gonna stick with the library's bad direct integration for now before we write our own, even worse one

            // Disable logging.
            imuParameters.loggingEnabled = false
            // Initialize IMU.
            imu?.initialize(imuParameters)
        }

        //         CHASSIS MOTOR POWER & DIRECTION CONFIG
        //                              F/B    L/R   TURN
        motorMap[frontright] = arrayOf(1.0, 1.0, -1.0)
        motorMap[backright] = arrayOf(1.0, -1.0, -1.0)
        motorMap[frontleft] = arrayOf(-1.0, 1.0, -1.0)
        motorMap[backleft] = arrayOf(-1.0, -1.0, -1.0)

        // Set all motors to zero power
        brake()
        closeClaw()
        closeClamp()
        shooter?.position = SHOOTER_IN
        armAtStartup()
    }

    fun hardwareLoop() {
        if (isImuEnabled) {
            rot = imu?.angularOrientation
            pos = imu?.position
        }
    }

    fun hardwareStop() {
        if (isImuEnabled) {
            imu?.stopAccelerationIntegration()
        }
    }

    fun startIMU(initPos: Position?) {
        imu?.startAccelerationIntegration(
                initPos,
                Velocity(DistanceUnit.METER, 0.0, 0.0, 0.0, 0),  // we don't need to worry about initial velocity. It's always going to be 0
                10 // maximum possible poll interval ~~ 100Hz sample rate
        )
    }

    fun chassis(joystick: DoubleArray) {
        for ((motor, values) in motorMap) {
            val power = values[0] * joystick[0] + values[1] * joystick[1] + values[2] * joystick[2]
            motor.power = power.coerceAtMost(1.0).coerceAtLeast(-1.0)
        }
    }

    fun closeClaw() {
        wobbleFinger?.position = CLOSE_CLAW
    }

    fun openClaw() {
        wobbleFinger?.position = OPEN_CLAW
    }

    fun closeClamp() {
        clamp?.position = CLOSE_CLAMP
    }

    fun openClamp() {
        clamp?.position = OPEN_CLAMP
    }

    fun armAtStartup() {
        wobble?.scaleRange(0.0, 1.0)
        wobble?.position = ARM_IN
        wobble?.scaleRange(ARM_OUT, ARM_MID)
    }

    fun armPower(d: Double) {
        wobble?.position = d
    }

    fun startIntake() {
        intake?.power = -1.0
    }

    fun stopIntake() {
        intake?.power = 0.0
    }

    fun reverseIntake() {
        intake?.power = 1.0
    }

    fun fire() {
        shooter?.position = SHOOTER_OUT
        Thread.sleep(1000)
        shooter?.position = SHOOTER_IN
    }

    fun startFlywheels() {
        leftFlywheel?.power = FLY1_POWER
        rightFlywheel?.power = FLY2_POWER
    }

    fun stopFlywheels() {
        leftFlywheel?.power = 0.0
        rightFlywheel?.power = 0.0
    }

    fun brake() {
        chassis(doubleArrayOf(0.0, 0.0, 0.0))
    }

    fun flywheelCurrentDraw(): Double {
        return (leftFlyMonitor!!.getCurrentDraw(AMPS) + rightFlyMonitor!!.getCurrentDraw(AMPS))
    }

    companion object {
        private const val CLOSE_CLAW = 0.65
        private const val OPEN_CLAW = 0.0
        private const val CLOSE_CLAMP = 0.0
        private const val OPEN_CLAMP = 0.65
        private const val ARM_IN = 0.95
        private const val ARM_MID = 0.6
        private const val ARM_OUT = 0.14
        const val INCHES_PER_SECOND = 32.0
        private const val FLY1_POWER = 1.0
        private const val FLY2_POWER = -1.0
        private const val SHOOTER_IN = 2.75
        private const val SHOOTER_OUT = 0.0
        private val AMPS = ExpansionHubEx.CurrentDrawUnits.AMPS
    }
}