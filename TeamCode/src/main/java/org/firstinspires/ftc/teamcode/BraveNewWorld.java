package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Map;
import java.util.HashMap;

@TeleOp(name = "Brave New World", group = "Pushbot")

public class BraveNewWorld extends OpMode {

    /* Declare OpMode members. */
    //AndroidOrientation compass(); // Android Orientation Object Declaration
    //AndroidTextToSpeech speek(); // Text to Speech Object
    @NonNull
    final
    RobotHardware robot = new RobotHardware(); // use the class created to define a Pushbot's hardware
    @NonNull
    final
    Map<DcMotor, Double[]> valueMap = new HashMap<>();
    double wobble_pos = 0;
    double finger_pos = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, "imu");

        //                                          F/B   L/R   TURN
        valueMap.put(robot.frontright, new Double[]{1.0,  1.0,  1.0});
        valueMap.put(robot.backright, new Double[]{-1.0,  1.0,  1.0});
        valueMap.put(robot.frontleft, new Double[]{-1.0,  1.0, -1.0});
        valueMap.put(robot.backleft, new Double[] { 1.0,  1.0, -1.0});
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Brave New World", "Groovy");
        telemetry.addData("Version", "0.21");
        telemetry.addData("Accelerometer", robot.imu.isAccelerometerCalibrated());
        telemetry.addData("Gyro", robot.imu.isGyroCalibrated());
        telemetry.addData("Magnetometer", robot.imu.isMagnetometerCalibrated());
        telemetry.addData("Calib. Status", robot.imu.getCalibrationStatus().toString());
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
                ,0
        ));
    }

    private void setPower(@NonNull DcMotor motor) {
        Double[] values = valueMap.get(motor);
        double forward_backward = values[0] * gamepad1.right_stick_y;
        double strafe = values[1] * gamepad1.right_stick_x;
        double turn = values[2] * gamepad1.left_stick_x;
        double power = forward_backward + strafe + turn;
        motor.setPower(Math.max(-1, Math.min(power, 1)));
    }

    @Override
    public void loop() {
        robot.hardware_loop();

        setPower(robot.frontright);
        setPower(robot.backright);
        setPower(robot.frontleft);
        setPower(robot.backleft);

        float[] hsv = {0,0,0};
        Color.colorToHSV(robot.greg.argb(), hsv);

        wobble_pos += 0.01 * gamepad2.left_stick_y;
        robot.wobble.setPosition(wobble_pos);
        finger_pos += 0.02 * gamepad2.left_stick_x;
        robot.wobbleFinger.setPosition(finger_pos);

        telemetry.addData("fwd/bkwd", "%.2f", gamepad1.right_stick_y);
        telemetry.addData("strafe", "%.2f", gamepad1.right_stick_x);
        telemetry.addData("turn", "%.2f\n------------", gamepad1.left_stick_x);
        telemetry.addData("Rot", "(%.2f, %.2f, %.2f)", robot.rot.thirdAngle, robot.rot.secondAngle, robot.rot.firstAngle);
        telemetry.addData("Pos", "(%.2fm, %.2fm, %.2fm)", robot.pos.x, robot.pos.y, robot.pos.z);
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
