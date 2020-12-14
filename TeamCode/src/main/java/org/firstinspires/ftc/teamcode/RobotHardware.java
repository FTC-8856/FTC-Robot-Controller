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

import android.location.Location;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.openftc.easyopencv.*;

/**
 * This is NOT an opmode.
 */
public class RobotHardware {
    /* Public OpMode members. */
    @Nullable
    public DcMotor frontleft = null;
    @Nullable
    public DcMotor frontright = null;
    @Nullable
    public DcMotor backleft = null;
    @Nullable
    public DcMotor backright = null;
    @Nullable
    public DcMotor intake = null;
    @Nullable
    public Servo wobble = null;
    @Nullable
    public Servo wobbleFinger = null;
    @Nullable
    public ColorSensor greg = null;
    @Nullable
    public OpenCvCamera webcam = null;

    /* IMU public variables */
    boolean isImuEnabled = false;
    BNO055IMU imu;
    BNO055IMU.Parameters imuParameters;
    Position pos;
    Orientation rot;

    /* local OpMode members. */
    @Nullable
    HardwareMap hwMap = null;
    private final ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RobotHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, String features) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontleft = hwMap.get(DcMotor.class, "frontleft");
        frontright = hwMap.get(DcMotor.class, "frontright");
        backleft = hwMap.get(DcMotor.class, "backleft");
        backright = hwMap.get(DcMotor.class, "backright");
        intake = hwMap.get(DcMotor.class, "intake"); // S c o o p s  the rings into the hopper to be shot at unsuspecting power shots and tower goals
        wobble = hwMap.get(Servo.class, "wobble"); // Wobble goal actuator arm rotator
        wobbleFinger = hwMap.get(Servo.class, "wobbleFinger"); // Wobble goal actuator arm "finger" grabber joint servo
        greg = hwMap.get(ColorSensor.class, "greg"); // Greg is our premier color sensor.
        //
        // This space has been left reserved for the firing mechanism devices when they are finished.
        //
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
        intake.setPower(0);
        wobble.setPosition(0);
        wobbleFinger.setPosition(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(features.contains("imu")){
            isImuEnabled = true;

            imu = hwMap.get(BNO055IMU.class, "imu");

            // Create new IMU Parameters object.
            imuParameters = new BNO055IMU.Parameters();
            imuParameters.mode = BNO055IMU.SensorMode.IMU;
            // Use degrees as angle unit.
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            // Express acceleration as m/s^2.
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            //imuParameters.accelerationIntegrationAlgorithm = ; // We're gonna stick with the library's bad direct integration for now before we write our own, even worse one

            // Disable logging.
            imuParameters.loggingEnabled = false;
            // Initialize IMU.
            imu.initialize(imuParameters);
        }

        // Do the OpenCV initialization if it is asked for
        // We will have to have this commented out until we figure out how to get the webcam on the hardware map
        //webcam = startCamera(hwMap.get(WebcamName.class, "logitech"));
    }

    /* We'll come back to this function
    public void hardware_start(){
        if(isImuEnabled) {
            imu.startAccelerationIntegration();
        }
    }
     */

    public void hardware_loop(){
        if(isImuEnabled) {
            rot = imu.getAngularOrientation();
            pos = imu.getPosition();
        }
    }

    public void hardware_stop(){
        if(isImuEnabled) {
            imu.stopAccelerationIntegration();
        }
    }

    public OpenCvCamera startCamera(WebcamName cameraID) { // Not done yet, this only gets the camera instance, but does not start the video streaming
        return OpenCvCameraFactory.getInstance().createWebcam(cameraID);
    }

    public void startIMU(Position init_pos){
        imu.startAccelerationIntegration(
                init_pos,
                new Velocity(DistanceUnit.METER, 0.0, 0.0, 0.0 , 0), // we don't need to worry about initial velocity. It's always going to be 0
                10 // maximum possible poll interval ~~ 100Hz sample rate
        );
    }
}

