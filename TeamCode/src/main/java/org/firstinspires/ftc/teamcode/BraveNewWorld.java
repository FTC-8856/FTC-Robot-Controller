package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@SuppressWarnings("unused")
@TeleOp(name = "Brave New World", group = "Pushbot")

public class BraveNewWorld extends OpMode {

    @NonNull
    private final
    RobotHardware robot = new RobotHardware(); // use the class created to define a Pushbot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, "imu");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Brave New World", "Groovy");
        telemetry.addData("Version", "0.21");
        telemetry.addData("Accelerometer", robot.getImu().isAccelerometerCalibrated());
        telemetry.addData("Gyro", robot.getImu().isGyroCalibrated());
        telemetry.addData("Magnetometer", robot.getImu().isMagnetometerCalibrated());
        telemetry.addData("Calib. Status", robot.getImu().getCalibrationStatus().toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.startIMU(new Position(
                DistanceUnit.METER,
                0.0, 0.0, 0.0 // <--- starting position
                , 0
        ));
    }

    @Override
    public void loop() {
        robot.hardwareLoop();

        robot.chassis(new double[]{gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x});

        float[] hsv = {0, 0, 0};
        Color.colorToHSV(robot.gregArgb(), hsv);

        robot.armPower((double) gamepad2.left_stick_y);
        if (gamepad2.right_bumper) {
            robot.openClaw();
        } else if (gamepad2.right_trigger > 0.1) {
            robot.closeClaw();
        }
        if (gamepad2.back) {
            robot.armStartup();
        }
        if (gamepad2.right_stick_button) {
            if (robot.areFlywheelsRunning()) {
                robot.stopFlywheels();
            } else {
                robot.startFlywheels();
            }
        }
        if (gamepad2.dpad_down) {
            robot.decrementFirePosition();
        }
        if (gamepad2.dpad_up) {
            robot.incrementFirePosition();
        }
        if (gamepad2.left_stick_button) {
            robot.fire();
        }
        if (gamepad2.a) {
            robot.startIntake();
        }
        if (gamepad2.b) {
            robot.reverseIntake();
        }
        if (gamepad2.x) {
            robot.stopIntake();
        }

        telemetry.addData("fwd/bkwd", "%.2f", gamepad1.right_stick_y);
        telemetry.addData("strafe", "%.2f", gamepad1.right_stick_x);
        telemetry.addData("turn", "%.2f\n------------", gamepad1.left_stick_x);
        telemetry.addData("Rot", "(%.2f, %.2f, %.2f)", robot.getRot().thirdAngle, robot.getRot().secondAngle, robot.getRot().firstAngle);
        telemetry.addData("Pos", "(%.2fm, %.2fm, %.2fm)", robot.getPos().x, robot.getPos().y, robot.getPos().z);
        telemetry.addData("HSV", "(%.2f, %.2f, %.2f)", hsv[0], hsv[1], hsv[2]);
        telemetry.addData("Fire position", "%s", robot.getFirePos());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.hardwareStop();
        telemetry.addData("Exit", "Goodest Good Job!");
        telemetry.update();
    }
}
