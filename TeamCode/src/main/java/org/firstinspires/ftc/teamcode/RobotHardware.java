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

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.HashMap;
import java.util.Map;

/**
 * This is NOT an opmode.
 */
public class RobotHardware {
    private static final Double CLOSE_CLAW = 0.6;
    private static final Double OPEN_CLAW = 0.0;
    private static final Double ARM_IN = 0.0;
    private static final Double MID_ARM = 0.3;
    private static final Double ARM_OUT = 0.95;
    private static final Double INTAKE_POWER = 1.0;
    private final ElapsedTime period = new ElapsedTime();
    @Nullable
    private final OpenCvCamera webcam = null;
    @NonNull
    private final Map<DcMotor, Double[]> motorMap = new HashMap<>();
    /* IMU public variables */
    private boolean isImuEnabled = false;
    private BNO055IMU imu;
    private BNO055IMU.Parameters imuParameters;
    private Position pos;
    private Orientation rot;
    /* local OpMode members. */
    @Nullable
    private HardwareMap hwMap = null;
    @Nullable
    private DcMotor intake = null;
    @Nullable
    private Servo wobble = null;
    @Nullable
    private Servo wobbleFinger = null;
    @Nullable
    private ColorSensor greg = null;

    /* Constructor */
    public RobotHardware() {
        // Does nothing
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, @NonNull String features) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        /* Public OpMode members. */
        DcMotor frontleft = hwMap.get(DcMotor.class, "frontleft");
        DcMotor frontright = hwMap.get(DcMotor.class, "frontright");
        DcMotor backleft = hwMap.get(DcMotor.class, "backleft");
        DcMotor backright = hwMap.get(DcMotor.class, "backright");
        intake = hwMap.get(DcMotor.class, "intake"); // S c o o p s  the rings into the hopper to be shot at unsuspecting power shots and tower goals
        wobble = hwMap.get(Servo.class, "wobble"); // Wobble goal actuator arm rotator
        wobbleFinger = hwMap.get(Servo.class, "wobbleFinger"); // Wobble goal actuator arm "finger" grabber joint servo
        greg = hwMap.get(ColorSensor.class, "greg"); // Greg is our premier color sensor.
        //
        // This space has been left reserved for the firing mechanism devices when they are finished.
        //
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (features.contains("imu")) {
            isImuEnabled = true;

            imu = hwMap.get(BNO055IMU.class, "imu");

            // Create new IMU Parameters object.
            imuParameters = new BNO055IMU.Parameters();
            imuParameters.mode = BNO055IMU.SensorMode.IMU;
            // Use degrees as angle unit.
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            // Express acceleration as m/s^2.
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imuParameters.calibrationDataFile = "GregIMUCalibration.json";
            //imuParameters.accelerationIntegrationAlgorithm = ; // We're gonna stick with the library's bad direct integration for now before we write our own, even worse one

            // Disable logging.
            imuParameters.loggingEnabled = false;
            // Initialize IMU.
            imu.initialize(imuParameters);
        }

        // Do the OpenCV initialization if it is asked for
        // We will have to have this commented out until we figure out how to get the webcam on the hardware map
        //webcam = startCamera(hwMap.get(WebcamName.class, "logitech"));

        //         CHASSIS MOTOR POWER & DIRECTION CONFIG
        //                                     F/B    L/R   TURN
        motorMap.put(frontright, new Double[]{-1.0, -0.935, -1.0});
        motorMap.put(backright, new Double[]{-1.0, 1.0, -1.0});
        motorMap.put(frontleft, new Double[]{-1.0, 0.935, 1.0});
        motorMap.put(backleft, new Double[]{-1.0, -1.0, 1.0});

        // Set all motors to zero power
        chassis(0, 0, 0);
        armStartup();
        closeClaw();
    }

    /* We'll come back to this function
    public void hardwareStart(){
        if(isImuEnabled) {
            imu.startAccelerationIntegration();
        }
    }
     */

    public void hardwareLoop() {
        if (isImuEnabled) {
            rot = imu.getAngularOrientation();
            pos = imu.getPosition();
        }
    }

    public void hardwareStop() {
        if (isImuEnabled) {
            imu.stopAccelerationIntegration();
        }
    }

    public OpenCvCamera startCamera(WebcamName cameraID) { // Not done yet, this only gets the camera instance, but does not start the video streaming
        return OpenCvCameraFactory.getInstance().createWebcam(cameraID);
    }

    public void startIMU(Position initPos) {
        imu.startAccelerationIntegration(
                initPos,
                new Velocity(DistanceUnit.METER, 0.0, 0.0, 0.0, 0), // we don't need to worry about initial velocity. It's always going to be 0
                10 // maximum possible poll interval ~~ 100Hz sample rate
        );
    }

    public void chassis(double rightStickY, double rightStickX, double leftStickX) {
        for (Map.Entry<DcMotor, Double[]> entry : motorMap.entrySet()) {
            Double[] values = entry.getValue();
            double power = values[0] * rightStickY + values[1] * rightStickX + values[2] * leftStickX;
            entry.getKey().setPower(Math.max(-1, Math.min(power, 1)));
        }
    }

    public void performAction(@NonNull Action action) {
        switch (action) {
            case CLOSE_CLAW:
                closeClaw();
                break;
            case OPEN_CLAW:
                openClaw();
                break;
            case RETRACT_ARM:
                armPower(0.0);
                break;
            case EXTEND_ARM:
                armPower(1.0);
                break;
            case START_INTAKE:
                startIntake();
                break;
            case STOP_INTAKE:
                stopIntake();
                break;
            case REVERSE_INTAKE:
                reverseIntake();
                break;
            case FIRE_LOW:
                fire(FirePosition.LOW);
                break;
            case FIRE_MID:
                fire(FirePosition.MEDIUM);
                break;
            case FIRE_HIGH:
                fire(FirePosition.HIGH);
                break;
            default:
                break;
        }
    }

    public void closeClaw() {
        wobbleFinger.setPosition(CLOSE_CLAW);
    }

    public void openClaw() {
        wobbleFinger.setPosition(OPEN_CLAW);
    }

    public void armStartup() {
        wobble.setPosition(ARM_IN);
    }

    public void armPower(Double d) {
        wobble.setPosition(MID_ARM + .5 * ((d + 1) * (ARM_OUT - MID_ARM)));
    }

    public void startIntake() {
        intake.setPower(INTAKE_POWER);
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    public void reverseIntake() {
        intake.setPower(-1 * INTAKE_POWER);
    }

    public void fire(FirePosition firePos) {
        if (firePos == FirePosition.HIGH) {
            // To-do
        }
        if (firePos == FirePosition.MEDIUM) {
            // To-do
        }
        if (firePos == FirePosition.LOW) {
            // To-do
        }
    }

    public void startFlywheels() {

    }

    public void stopFlywheels() {

    }

    public boolean areFlywheelsRunning() {
        return true;
    }

    public int gregArgb() {
        return greg.argb();
    }

    public BNO055IMU getImu() {
        return this.imu;
    }

    public Position getPos() {
        return this.pos;
    }

    public Orientation getRot() {
        return this.rot;
    }
}

