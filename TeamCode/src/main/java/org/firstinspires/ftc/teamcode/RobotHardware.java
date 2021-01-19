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

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
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
    private static final Double INCHES_PER_SECOND = 52.5;
    @SuppressWarnings("unused")
    private final ElapsedTime period = new ElapsedTime();
    @NonNull
    private final Map<DcMotor, Double[]> motorMap = new HashMap<>();
    @SuppressWarnings("unused")
    @Nullable
    private WebcamName webcam = null;
    /* IMU public variables */
    private boolean isImuEnabled = false;
    private BNO055IMU imu;
    private Position pos;
    private Orientation rot;
    @Nullable
    private DcMotor intake = null;
    @Nullable
    private Servo wobble = null;
    @Nullable
    private Servo wobbleFinger = null;
    @Nullable
    private ColorSensor greg = null;
    @NonNull
    private FirePosition firePosition = FirePosition.MEDIUM;

    /* Constructor */
    @SuppressWarnings("unused")
    public RobotHardware() {
        // Does nothing
    }

    /* Initialize standard Hardware interfaces */
    public void init(@NonNull HardwareMap ahwMap, @NonNull String features) {
        // Save reference to Hardware map
        /* local OpMode members. */

        // Define and Initialize Motors
        /* Public OpMode members. */
        DcMotor frontleft = ahwMap.get(DcMotor.class, "frontleft");
        DcMotor frontright = ahwMap.get(DcMotor.class, "frontright");
        DcMotor backleft = ahwMap.get(DcMotor.class, "backleft");
        DcMotor backright = ahwMap.get(DcMotor.class, "backright");
        intake = ahwMap.get(DcMotor.class, "intake"); // S c o o p s  the rings into the hopper to be shot at unsuspecting power shots and tower goals
        wobble = ahwMap.get(Servo.class, "wobble"); // Wobble goal actuator arm rotator
        wobbleFinger = ahwMap.get(Servo.class, "wobbleFinger"); // Wobble goal actuator arm "finger" grabber joint servo
        greg = ahwMap.get(ColorSensor.class, "greg"); // Greg is our premier color sensor.
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
        if (intake != null) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (features.contains("imu")) {
            isImuEnabled = true;

            imu = ahwMap.get(BNO055IMU.class, "imu");

            // Create new IMU Parameters object.
            BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
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
        webcam = ahwMap.get(WebcamName.class, "gregcam");

        //         CHASSIS MOTOR POWER & DIRECTION CONFIG
        //                                     F/B    L/R   TURN
        motorMap.put(frontright, new Double[]{-1.0, 0.935, -1.0});
        motorMap.put(backright, new Double[]{-1.0, -1.0, -1.0});
        motorMap.put(frontleft, new Double[]{-1.0, -0.935, 1.0});
        motorMap.put(backleft, new Double[]{-1.0, 1.0, 1.0});

        // Set all motors to zero power
        chassis(new double[]{0, 0, 0});
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

    @SuppressWarnings("unused")
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

    public void chassis(double[] joystick) {
        for (Map.Entry<DcMotor, Double[]> entry : motorMap.entrySet()) {
            Double[] values = entry.getValue();
            double power = values[0] * joystick[0] + values[1] * joystick[1] + values[2] * joystick[2];
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
                firePosition = FirePosition.LOW;
                fire();
                break;
            case FIRE_MID:
                firePosition = FirePosition.MEDIUM;
                fire();
                break;
            case FIRE_HIGH:
                firePosition = FirePosition.HIGH;
                fire();
                break;
            default:
                break;
        }
    }

    public void closeClaw() {
        if (wobbleFinger != null) {
            wobbleFinger.setPosition(CLOSE_CLAW);
        }
    }

    public void openClaw() {
        if (wobbleFinger != null) {
            wobbleFinger.setPosition(OPEN_CLAW);
        }
    }

    public void armStartup() {
        if (wobble != null) {
            wobble.setPosition(ARM_IN);
        }
    }

    public void armPower(Double d) {
        if (wobble != null) {
            double position = MID_ARM + .5 * ((d + 1) * (ARM_OUT - MID_ARM));
            wobble.setPosition(position);
        }
    }

    public void startIntake() {
        if (intake != null) {
            intake.setPower(INTAKE_POWER);
        }
    }

    public void stopIntake() {
        if (intake != null) {
            intake.setPower(0);
        }
    }

    public void reverseIntake() {
        if (intake != null) {
            intake.setPower(-1 * INTAKE_POWER);
        }
    }

    public void fire() {
        if (firePosition == FirePosition.HIGH) {
            // To-do
        }
        if (firePosition == FirePosition.MEDIUM) {
            // To-do
        }
        if (firePosition == FirePosition.LOW) {
            // To-do
        }
    }

    public void decrementFirePosition() {
        if (firePosition == FirePosition.HIGH) {
            firePosition = FirePosition.MEDIUM;
        } else {
            firePosition = FirePosition.LOW;
        }
    }

    public void incrementFirePosition() {
        if (firePosition == FirePosition.LOW) {
            firePosition = FirePosition.MEDIUM;
        } else {
            firePosition = FirePosition.HIGH;
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
        if (greg != null) {
            return greg.argb();
        } else {
            return 0;
        }
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

    @NonNull
    public FirePosition getFirePos() {
        return this.firePosition;
    }

    public void waitFor(long ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception ignored) {
        }
        chassis(new double[]{0, 0, 0});
    }


    public void driveFor(Double inches) {
        chassis(new double[]{-1.0, 0, 0});
        waitFor((long) (1000 * inches / INCHES_PER_SECOND));
    }

    public WebcamName getWebcam() {
        return this.webcam;
    }
}

