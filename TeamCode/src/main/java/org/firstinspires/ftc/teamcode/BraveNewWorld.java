package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "Brave New World", group = "Pushbot")

public class BraveNewWorld extends OpMode {

    @NonNull
    final
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
        telemetry.addData("Accelerometer", robot.get_imu().isAccelerometerCalibrated());
        telemetry.addData("Gyro", robot.get_imu().isGyroCalibrated());
        telemetry.addData("Magnetometer", robot.get_imu().isMagnetometerCalibrated());
        telemetry.addData("Calib. Status", robot.get_imu().getCalibrationStatus().toString());
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
        robot.hardware_loop();

        robot.chassis(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        float[] hsv = {0, 0, 0};
        Color.colorToHSV(robot.greg_argb(), hsv);

        if (gamepad2.left_stick_y > 0.1) {
            robot.retract_arm();
        } else if (gamepad2.left_stick_y < -0.1) {
            robot.extend_arm();
        }
        if (gamepad2.left_stick_x > 0.1) {
            robot.open_claw();
        } else if (gamepad2.left_stick_x < -0.1) {
            robot.close_claw();
        }
        if (gamepad1.y) {
            robot.fire_high();
        }
        if (gamepad1.x) {
            robot.fire_mid();
        }
        if (gamepad1.a) {
            robot.fire_low();
        }
        if (gamepad2.a) {
            robot.start_intake();
        }
        if (gamepad2.b) {
            robot.reverse_intake();
        }
        if (gamepad2.x) {
            robot.stop_intake();
        }

        telemetry.addData("fwd/bkwd", "%.2f", gamepad1.right_stick_y);
        telemetry.addData("strafe", "%.2f", gamepad1.right_stick_x);
        telemetry.addData("turn", "%.2f\n------------", gamepad1.left_stick_x);
        telemetry.addData("Rot", "(%.2f, %.2f, %.2f)", robot.get_rot().thirdAngle, robot.get_rot().secondAngle, robot.get_rot().firstAngle);
        telemetry.addData("Pos", "(%.2fm, %.2fm, %.2fm)", robot.get_pos().x, robot.get_pos().y, robot.get_pos().z);
        telemetry.addData("HSV", "(%.2f, %.2f, %.2f)", hsv[0], hsv[1], hsv[2]);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.hardware_stop();
        telemetry.addData("Exit", "Goodest Good Job!");
        telemetry.update();
    }
}
