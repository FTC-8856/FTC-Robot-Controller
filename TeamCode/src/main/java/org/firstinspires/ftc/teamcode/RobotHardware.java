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

import android.os.Build;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * This is NOT an opmode.
 */
@RequiresApi(api = Build.VERSION_CODES.N)
@SuppressWarnings("unused")
public class RobotHardware {
    private static final double INCHES_PER_SECOND = 52.5;
    private static final double MILLISECONDS_PER_SECONDS = 1000.0;

    @SuppressWarnings("unused")
    @Nullable
    private final Optional<OpenCvCamera> webcam = Optional.empty();
    @NonNull
    private final Map<DcMotor, double[]> motorMap = new HashMap<>(4);
    /* IMU public variables */
    private boolean isImuEnabled = false;
    private BNO055IMU imu = null;
    private Position pos = null;
    private Orientation rot = null;
    private Optional<DcMotor> intake = Optional.empty();
    private Optional<Servo> wobble = Optional.empty();
    private Optional<Servo> wobbleFinger = Optional.empty();
    @Nullable
    private ColorSensor greg;
    private Optional<Servo> shooter = Optional.empty();

    @SuppressWarnings("unused")
    @NonNull
    public static OpenCvCamera startCamera(@NonNull final WebcamName cameraID) {
        // Not done yet, this only gets the camera instance, but does not start the video streaming
        return OpenCvCameraFactory.getInstance().createWebcam(cameraID);
    }

    private static void waitFor(final long ms) {
        try {
            Thread.sleep(ms);
        } catch (final InterruptedException e) {
            e.printStackTrace();
            Thread.currentThread().interrupt();
        }
    }

    private static long inchesToMs(final double inches) {
        final double seconds = inches / RobotHardware.INCHES_PER_SECOND;
        return (long) (RobotHardware.MILLISECONDS_PER_SECONDS * seconds);
    }

    final void fire(@NonNull final FirePosition firePos) {
        if (this.shooter.isPresent()) {
            final Servo servo = this.shooter.get();
            final double position = firePos.toServoPosition();
            servo.setPosition(position);
        }
    }

    @SuppressWarnings("SameReturnValue")
    boolean areFlywheelsRunning() {
        return true;
    }

    /* Initialize standard Hardware interfaces */
    final void init(@NonNull final HardwareMap ahwMap, @NonNull final String features) {
        // Save reference to Hardware map
        /* local OpMode members. */

        // Define and Initialize Motors
        /* Public OpMode members. */
        final DcMotor frontleft = ahwMap.get(DcMotor.class, "frontleft");
        final DcMotor frontright = ahwMap.get(DcMotor.class, "frontright");
        final DcMotor backleft = ahwMap.get(DcMotor.class, "backleft");
        final DcMotor backright = ahwMap.get(DcMotor.class, "backright");
        final DcMotor nullableIntake = ahwMap.get(DcMotor.class, "intake");
        this.intake = Optional.of(nullableIntake); // S c o o p s  the rings
        // into the hopper to be shot at unsuspecting power shots and tower goals
        final Servo nullableWobble = ahwMap.get(Servo.class, "wobble");
        this.wobble = Optional.of(nullableWobble);
        final Servo nullableWobbleFinger = ahwMap.get(Servo.class, "wobbleFinger");
        this.wobbleFinger = Optional.of(nullableWobbleFinger); // Wobble goal
        // actuator arm "finger" grabber joint servo
        this.greg = ahwMap.get(ColorSensor.class, "greg"); // Greg is our premier
        // color sensor.
        //
        // This space has been left reserved for the firing mechanism devices when they are finished.
        //
        final Servo nullableShooter = ahwMap.get(Servo.class, "shooter");
        this.shooter = Optional.of(nullableShooter);

        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (this.intake.isPresent()) {
            final DcMotor dcMotor = this.intake.get();
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (features.contains("imu")) {
            this.isImuEnabled = true;

            this.imu = ahwMap.get(BNO055IMU.class, "imu");

            // Create new IMU Parameters object.
            final BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
            imuParameters.mode = BNO055IMU.SensorMode.IMU;
            // Use degrees as angle unit.
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            // Express acceleration as m/s^2.
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imuParameters.calibrationDataFile = "GregIMUCalibration.json";
            //imuParameters.accelerationIntegrationAlgorithm = ; // We're gonna stick with the
            // library's bad direct integration for now before we write our own, even worse one

            // Disable logging.
            imuParameters.loggingEnabled = false;
            // Initialize IMU.
            this.imu.initialize(imuParameters);
        }

        // Do the OpenCV initialization if it is asked for
        // We will have to have this commented out until we figure out how to get the webcam on the
        // hardware map

        //         CHASSIS MOTOR POWER & DIRECTION CONFIG
        //                                     F/B    L/R   TURN
        this.motorMap.put(frontright, new double[]{-1.0, -0.935, -1.0});
        this.motorMap.put(backright, new double[]{-1.0, 1.0, -1.0});
        this.motorMap.put(frontleft, new double[]{-1.0, 0.935, 1.0});
        this.motorMap.put(backleft, new double[]{-1.0, -1.0, 1.0});

        // Set all motors to zero power
        this.chassis(new double[]{0.0, 0.0, 0.0});
        this.armStartup();
        this.closeClaw();
    }

    final void hardwareLoop() {
        if (this.isImuEnabled) {
            this.rot = this.imu.getAngularOrientation();
            this.pos = this.imu.getPosition();
        }
    }

    final void hardwareStop() {
        if (this.isImuEnabled) {
            this.imu.stopAccelerationIntegration();
        }
    }

    final void startIMU(@NonNull final Position initPos) {
        this.imu.startAccelerationIntegration(
                initPos,
                new Velocity(DistanceUnit.METER, 0.0, 0.0, 0.0,
                        0L), // we don't need to worry about initial velocity. It's
                // always going to be 0
                10 // maximum possible poll interval ~~ 100Hz sample rate
        );
    }

    final void chassis(@NonNull final double[] joystick) {
        for (final Map.Entry<DcMotor, double[]> entry : this.motorMap.entrySet()) {
            final double[] values = entry.getValue();
            final double power = values[0] * joystick[0] + values[1] * joystick[1] + values[2] *
                    joystick[2];
            final DcMotor key = entry.getKey();
            final double min = Math.min(power, 1.0);
            final double max = Math.max(-1.0, min);
            key.setPower(max);
        }
    }

    final void performAction(@NonNull final Action action) {
        switch (action) {
            case CLOSE_CLAW:
                this.closeClaw();
                break;
            case OPEN_CLAW:
                this.openClaw();
                break;
            case RETRACT_ARM:
                this.armPower(0.0);
                break;
            case EXTEND_ARM:
                this.armPower(1.0);
                break;
            case START_INTAKE:
                this.startIntake();
                break;
            case STOP_INTAKE:
                this.stopIntake();
                break;
            case REVERSE_INTAKE:
                this.reverseIntake();
                break;
            case FIRE_LOW:
                this.fire(FirePosition.LOW);
                break;
            case FIRE_MID:
                this.fire(FirePosition.MEDIUM);
                break;
            case FIRE_HIGH:
                this.fire(FirePosition.HIGH);
                break;
            default:
                break;
        }
    }

    final void closeClaw() {
        if (this.wobbleFinger.isPresent()) {
            final Servo servo = this.wobbleFinger.get();
            servo.setPosition(0.6);
        }
    }

    final void openClaw() {
        if (this.wobbleFinger.isPresent()) {
            final Servo servo = this.wobbleFinger.get();
            servo.setPosition(0.0);
        }
    }

    final void armStartup() {
        if (this.wobble.isPresent()) {
            final Servo servo = this.wobble.get();
            servo.setPosition(0.0);
        }
    }

    final void armPower(final double input) {
        if (this.wobble.isPresent()) {
            final Servo servo = this.wobble.get();
            final double position = RobotHardware.normalize(input,
                    1.0, -1.0, 0.95, 0.3);
            servo.setPosition(position);
        }
    }

    @SuppressWarnings("SameParameterValue")
    private static double normalize(final double input,
                                    final double inputMax, final double inputMin,
                                    final double outputMax, final double outputMin) {
        final double inputDiff = inputMax - inputMin;
        final double outputDiff = outputMax - outputMin;
        final double inputOffset = input - inputMin;
        return outputMin + (inputOffset / inputDiff) * outputDiff;
    }

    final void startIntake() {
        if (this.intake.isPresent()) {
            final DcMotor dcMotor = this.intake.get();
            dcMotor.setPower(1.0);
        }
    }

    final void stopIntake() {
        if (this.intake.isPresent()) {
            final DcMotor dcMotor = this.intake.get();
            dcMotor.setPower(0.0);
        }
    }

    final void reverseIntake() {
        if (this.intake.isPresent()) {
            final DcMotor dcMotor = this.intake.get();
            dcMotor.setPower(-1.0 * 1.0);
        }
    }

    void startFlywheels() {
        // To-do
    }

    void stopFlywheels() {
        // To-do
    }

    final int gregArgb() {
        final ColorSensor colorSensor = Objects.requireNonNull(this.greg);
        return colorSensor.argb();
    }

    @NonNull
    final BNO055IMU getImu() {
        return this.imu;
    }

    @NonNull
    final Position getPos() {
        return this.pos;
    }

    @NonNull
    final Orientation getRot() {
        return this.rot;
    }

    private void waitForAdjusted(final long ms, final double[] adjusted) {
        RobotHardware.waitFor(ms);
        this.chassis(adjusted);
    }

    final void driveFor(final double inches, @NonNull final double[] joystick) {
        this.chassis(joystick);
        final long ms = RobotHardware.inchesToMs(inches);
        RobotHardware.waitFor(ms);
    }

    final void driveForAdjusted(final double inches, @NonNull final double[] joystick, @NonNull final double[] adjusted) {
        this.chassis(joystick);
        final long ms = RobotHardware.inchesToMs(inches);
        this.waitForAdjusted(ms, adjusted);
    }
}

