package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@SuppressWarnings("unused")
@TeleOp(name = "Brave New World", group = "Pushbot")
class BraveNewWorld extends OpMode {

    @NonNull
    private final
    RobotHardware robot = new RobotHardware();
    @NonNull
    private
    FirePosition firePos = FirePosition.LOW;

    BraveNewWorld() {
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public final void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        this.robot.init(this.hardwareMap, "imu");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public final void init_loop() {
        this.telemetry.addData("Brave New World", "Groovy");
        this.telemetry.addData("Version", "0.21");
        final BNO055IMU imu = this.robot.getImu();
        final boolean accelCalibrated = imu.isAccelerometerCalibrated();
        this.telemetry.addData("Accelerometer",
                accelCalibrated);
        final boolean gyroCalibrated = imu.isGyroCalibrated();
        this.telemetry.addData("Gyro", gyroCalibrated);
        final boolean magnetCalibrated = imu.isMagnetometerCalibrated();
        this.telemetry.addData("Magnetometer",
                magnetCalibrated);
        final BNO055IMU.CalibrationStatus calibrationStatus = imu.getCalibrationStatus();
        final String value = calibrationStatus.toString();
        this.telemetry.addData("Calib. Status", value);
        this.telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public final void start() {
        this.robot.startIMU(new Position(
                DistanceUnit.METER,
                0.0, 0.0, 0.0 // <--- starting position
                , 0L
        ));
    }

    @Override
    public final void loop() {
        this.robot.hardwareLoop();

        this.robot.chassis(new double[]{(double) this.gamepad1.right_stick_y, (double) this.gamepad1.right_stick_x, (double) this.gamepad1.left_stick_x});

        final float[] hsv = {(float) 0, (float) 0, (float) 0};
        final int argb = this.robot.gregArgb();
        Color.colorToHSV(argb, hsv);

        this.robot.armPower(this.gamepad2.left_stick_y);
        if (this.gamepad2.right_bumper) {
            this.robot.openClaw();
        } else if (0.1 < (double) this.gamepad2.right_trigger) {
            this.robot.closeClaw();
        }
        if (this.gamepad2.back) {
            this.robot.armStartup();
        }
        if (this.gamepad2.right_stick_button) {
            if (RobotHardware.areFlywheelsRunning()) {
                this.robot.stopFlywheels();
            } else {
                this.robot.startFlywheels();
            }
        }
        if (this.gamepad2.dpad_down) {
            if (FirePosition.LOW == this.firePos) {
                this.firePos = FirePosition.MEDIUM;
            } else {
                this.firePos = FirePosition.HIGH;
            }
        }
        if (this.gamepad2.dpad_up) {
            if (FirePosition.HIGH == this.firePos) {
                this.firePos = FirePosition.MEDIUM;
            } else {
                this.firePos = FirePosition.LOW;
            }
        }
        if (this.gamepad2.left_stick_button) {
            RobotHardware.fire(this.firePos);
        }
        if (this.gamepad2.a) {
            this.robot.startIntake();
        }
        if (this.gamepad2.b) {
            this.robot.reverseIntake();
        }
        if (this.gamepad2.x) {
            this.robot.stopIntake();
        }

        this.telemetry.addData("fwd/bkwd", "%.2f", this.gamepad1.right_stick_y);
        this.telemetry.addData("strafe", "%.2f", this.gamepad1.right_stick_x);
        this.telemetry.addData("turn", "%.2f\n------------", this.gamepad1.left_stick_x);
        final Orientation rot = this.robot.getRot();
        this.telemetry.addData("Rot", "(%.2f, %.2f, %.2f)", rot.thirdAngle, rot.secondAngle, rot.firstAngle);
        final Position pos = this.robot.getPos();
        this.telemetry.addData("Pos", "(%.2fm, %.2fm, %.2fm)",
                pos.x, pos.y, pos.z);
        this.telemetry.addData("HSV", "(%.2f, %.2f, %.2f)", hsv[0], hsv[1], hsv[2]);
        this.telemetry.addData("Fire position", "%s", this.firePos);
        this.telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public final void stop() {
        this.robot.hardwareStop();
        this.telemetry.addData("Exit", "Goodest Good Job!");
        this.telemetry.update();
    }
}
